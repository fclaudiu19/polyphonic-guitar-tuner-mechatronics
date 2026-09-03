/* ============================================================================
   ACORDOR AUTOMAT DE CHITARA — ESP32-S3 + Nanotec C5-01 + MAX4466
   ----------------------------------------------------------------------------
   Versiune prelucrata / optimizata.

   Imbunatatiri fata de varianta initiala:
     1. Auto-calibrare offset DC microfon la pornire (in loc de 2007 fix).
     2. Configurare explicita ADC (rezolutie 12 bit + atenuare 11dB).
     3. SAMPLES marit la 1024 -> rezolutie spectrala mai buna (interpolare).
     4. Detectie fundamentala robusta cu HPS (Harmonic Product Spectrum)
        -> elimina erorile de octava la corzile groase (E2, A2).
     5. Cautarea varfului doar in BANDA corzii selectate -> mult mai stabil.
     6. Control directie motor robust: flag INVERT_DIR, timp setup/hold DIR
        garantat, DIR nu se schimba in timpul unui burst de pasi.
     7. Endpoint /test pentru verificarea fizica a sensului motorului.
     8. Stepping non-blocant: server-ul web e servit si in timpul rotirii.
     9. Management EN: driver activat la miscare, dezactivat cand e oprit.
    10. Masina de stari clara + confirmare acordaj (anti fals-lock).
    11. JSON status fara diacritice -> fara probleme de encoding.

   Hardware (vezi conexiuni_finale.md):
     MIC_PIN  = GPIO7  (MAX4466 OUT, ADC1 — merge cu WiFi pornit)
     STEP_PIN = GPIO18 (C5-01 Input 6 Clock)
     DIR_PIN  = GPIO4  (C5-01 Input 5 Direction)
     EN_PIN   = GPIO5  (C5-01 Input 4 Enable, LOW = activ)

   NOTA HARDWARE despre directie:
     Intrarile C5-01 sunt optocuplate. ESP32 da 3.3V pe DIR/STEP/EN.
     Daca dupa aceste corectii motorul tot merge intr-un singur sens,
     cauza e aproape sigur HARDWARE: 3.3V e marginal pentru optocuplorul
     de directie. Solutii: alimenteaza intrarea "+" prin 5V (de la LM2596)
     printr-un tranzistor NPN / level-shifter comandat de GPIO, SAU
     verifica referinta comuna GND intre ESP32 si C5-01 X4 GND.
============================================================================ */

#include <WiFi.h>
#include <WebServer.h>
#include <arduinoFFT.h>

// ─────────────────────────── WiFi ───────────────────────────
const char* ssid     = "DIGI-tsCk";
const char* password = "PekZTZfCQf";

// ── IP static (placa are mereu aceeasi adresa -> o deschizi direct in browser) ──
// ATENTIE: valabil DOAR pe reteaua 192.168.1.x. Pe alt router / hotspot cu alt
// subnet (ex: 192.168.43.x), pune USE_STATIC_IP = false (revine la IP automat).
const bool USE_STATIC_IP = true;
IPAddress ip_static (192, 168, 1, 147);   // adresa fixa a placii
IPAddress ip_gateway(192, 168, 1, 1);     // de obicei routerul
IPAddress ip_subnet (255, 255, 255, 0);
IPAddress ip_dns    (192, 168, 1, 1);

// ─────────────────────────── Pini ───────────────────────────
#define MIC_PIN     7
#define STEP_PIN    18
#define DIR_PIN     4
#define EN_PIN      5

// ─────────────────────── FFT / esantionare ──────────────────
// SAMPLES=2048 -> rezolutie ~3.9 Hz/bin, ~256 ms/cadru (echilibru bun
// precizie/raspuns UI). Pt maxima precizie la E2/A2 poti pune 4096 (bin ~1.95 Hz)
// dar interfata devine mai lenta la STOP / schimbare coarda.
#define SAMPLES          2048
#define SAMPLE_RATE      8000          // Hz (Nyquist 4000 — acopera tot + armonice HPS)
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_RATE)   // 125 us

double vReal[SAMPLES];
double vImag[SAMPLES];
double hps[SAMPLES / 2];               // spectru pentru Harmonic Product Spectrum

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLE_RATE);

WebServer server(80);

// ─────────────────────── Tinte acordaj ──────────────────────
double frecventeTinta[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63};
String numeCorde[]      = {"E2",  "A2",   "D3",   "G3",   "B3",   "E4"};
int    coardaSelectata  = 1;

// ─────────────────────── Control motor ──────────────────────
// Timing pas (semi-perioada pulsului). 1/(2*STEP_DELAY) ~= frecventa clock.
#define STEP_DELAY     3500            // us — de ajustat la testare
// Timp de stabilizare a semnalului DIR inainte/dupa burst (setup/hold).
#define DIR_SETUP_US   200             // us — DIR stabil inainte de primul puls
// Inversare sens daca motorul "strange" cand ar trebui sa "slabeasca".
// Schimba in true daca, la test (butoanele din interfata), sensul e invers.
const bool INVERT_DIR = false;
// Tine motorul mereu energizat (cuplu de mentinere), ca in codul original.
// Pune false daca vrei sa-l lasi liber cand nu acordeaza (mai putina caldura).
const bool TINE_MEREU_TENSIUNE = true;
// Polaritatea intrarii ENABLE a driverului.
// C5-01 (Nanotec): Enable e activ pe HIGH -> EN_ACTIV_LOW = false.
// TMC2209: Enable era activ pe LOW -> EN_ACTIV_LOW = true.
// Daca L1 ramane verde clipind dupa upload, comuta aceasta valoare.
const bool EN_ACTIV_LOW = false;
// Calibrare: cati pasi schimba frecventa cu 1 cent (de reverificat experimental).
#define PASI_PER_CENT  50.0
// La cate microsecunde de stepping servim si clientul web (non-blocant).
#define STEP_BURST_SERVICE 10          // serveste webserver des -> STOP/UI responsiv

// Limita de siguranta — peste asta oprim automat.
double EROARE_MAX_CENTI = 300.0;

// ─────────────────────── Offset DC microfon ─────────────────
double offsetDC = 2007.0;              // valoare de start, recalibrata in setup()

// ─────────────────────── Filtru median ──────────────────────
#define MEDIAN_N 5
double bufferMedian[MEDIAN_N] = {0, 0, 0, 0, 0};
int    indexMedian            = 0;
bool   bufferPlin             = false;

double filtruMedian(double valoare) {
  bufferMedian[indexMedian] = valoare;
  indexMedian = (indexMedian + 1) % MEDIAN_N;
  if (indexMedian == 0) bufferPlin = true;
  int n = bufferPlin ? MEDIAN_N : indexMedian;
  double temp[MEDIAN_N];
  for (int i = 0; i < n; i++) temp[i] = bufferMedian[i];
  for (int i = 0; i < n - 1; i++)
    for (int j = 0; j < n - i - 1; j++)
      if (temp[j] > temp[j + 1]) {
        double t = temp[j]; temp[j] = temp[j + 1]; temp[j + 1] = t;
      }
  return temp[n / 2];
}

void resetMedian() {
  for (int i = 0; i < MEDIAN_N; i++) bufferMedian[i] = 0;
  indexMedian = 0;
  bufferPlin  = false;
}

// ─────────────────────── Filtru IIR ─────────────────────────
double alfa    = 0.7;
double iirPrev = 0.0;
bool   iirInit = false;

double filtruIIR(double valoare) {
  if (!iirInit) { iirPrev = valoare; iirInit = true; return valoare; }
  iirPrev = alfa * valoare + (1.0 - alfa) * iirPrev;
  return iirPrev;
}

void resetIIR() { iirInit = false; iirPrev = 0.0; }

// ─────────────────────── Filtru Kalman ──────────────────────
double Q = 0.1, R = 1.0, P = 1.0, x = 0.0;
bool   kalmanInitializat = false;

double filtruKalman(double masurare) {
  if (!kalmanInitializat || abs(masurare - x) > 25.0) {
    x = masurare; P = 1.0; kalmanInitializat = true;
    return x;
  }
  double x_pred = x;
  double P_pred = P + Q;
  double K      = P_pred / (P_pred + R);
  x = x_pred + K * (masurare - x_pred);
  P = (1.0 - K) * P_pred;
  return x;
}

// ─────────────────────── Stare sistem ───────────────────────
double kalPrev      = 0;
int    stabilCount  = 0;
int    acordatCount = 0;               // confirmari consecutive "in toleranta"

bool   sistemActiv  = false;
double frecvCurenta = 0.0;
double eroareCurenta= 0.0;
String statusCurent = "Asteapta...";

bool   driverActiv  = false;           // urmareste starea EN
volatile bool abortMiscare = false;    // STOP intrerupe un burst de pasi in curs
double rawCurenta = 0;                 // ultima frecventa bruta (diagnostic/log)
double magCurenta = 0;                 // ultima magnitudine varf (diagnostic/log)
long   pasiTotal  = 0;                 // pasi cumulati: + strange, - slabeste (diagnostic)

// ─────────── Parametri precizie / control buclă (de reglat) ───────────
const double TOL_CENTI      = 5.0;     // zona "ACORDATA" (±)
const int    STABIL_N       = 3;       // cadre necesare pt o masuratoare STABILA
const double STABIL_SPREAD  = 3.0;     // cenți: imprastiere max in buffer ca sa fie "stabil"
const double STEPS_PER_CENT = 8.0;     // pasi comandati / cent eroare (SUB-corectie -> fara overshoot)
const int    PASI_MAX       = 400;     // plafon pasi/corectie
const int    PASI_MIN       = 2;       // minim pasi/corectie
const unsigned long SETTLE_MS = 250;   // pauza dupa o miscare (coarda+mecanica se aseaza)
const int    BACKLASH_PASI  = 0;       // compensare joc la SCHIMBAREA sensului (regleaza experimental)

// Buffer de stabilitate (in cenți fata de tinta) — decidem doar pe citiri stabile.
double stabilBuf[STABIL_N];
int    stabilIdx = 0, stabilCnt = 0;
unsigned long ultimaMiscare = 0;
bool   ultimaDirStrange = true;        // pt compensarea backlash

void resetStabil() { stabilIdx = 0; stabilCnt = 0; }

void pushStabil(double c) {
  stabilBuf[stabilIdx] = c;
  stabilIdx = (stabilIdx + 1) % STABIL_N;
  if (stabilCnt < STABIL_N) stabilCnt++;
}

bool stabilGata() {
  if (stabilCnt < STABIL_N) return false;
  double mn = stabilBuf[0], mx = stabilBuf[0];
  for (int i = 1; i < STABIL_N; i++) {
    if (stabilBuf[i] < mn) mn = stabilBuf[i];
    if (stabilBuf[i] > mx) mx = stabilBuf[i];
  }
  return (mx - mn) <= STABIL_SPREAD;
}

double medianaStabil() {
  double t[STABIL_N];
  for (int i = 0; i < STABIL_N; i++) t[i] = stabilBuf[i];
  for (int i = 0; i < STABIL_N - 1; i++)
    for (int j = 0; j < STABIL_N - 1 - i; j++)
      if (t[j] > t[j + 1]) { double s = t[j]; t[j] = t[j + 1]; t[j + 1] = s; }
  return t[STABIL_N / 2];
}

// Toate cadrele colectate au acelasi semn (aceeasi directie de corectie)?
// Folosit ca sa miscam fiabil fara sa actionam pe un singur cadru-glitch.
bool acelasiSemn() {
  if (stabilCnt < 2) return false;
  bool poz = stabilBuf[0] > 0;
  for (int i = 1; i < stabilCnt; i++)
    if ((stabilBuf[i] > 0) != poz) return false;
  return true;
}

// ─────────────────────── Driver enable ──────────────────────
void enableDriver(bool on) {
  if (!on && TINE_MEREU_TENSIUNE) on = true;   // nu lasam motorul fara tensiune
  if (on == driverActiv) return;
  int nivelActiv = EN_ACTIV_LOW ? LOW : HIGH;
  digitalWrite(EN_PIN, on ? nivelActiv : !nivelActiv);
  driverActiv = on;
  if (on) delay(2);                        // mic timp de wake-up driver
}

// ─────────────────────── Calibrare offset DC ────────────────
void calibreazaOffsetDC() {
  const int N = 2000;
  double suma = 0;
  for (int i = 0; i < N; i++) { suma += analogRead(MIC_PIN); delayMicroseconds(100); }
  offsetDC = suma / N;
  Serial.print("Offset DC microfon calibrat: ");
  Serial.println(offsetDC, 1);
}

// ─────────────────── Detectie fundamentala (HPS) ────────────
// Cauta fundamentala in banda corzii selectate folosind Harmonic Product
// Spectrum (inmulteste spectrul cu versiuni decimate 2x, 3x). Astfel un
// armonic puternic NU mai e confundat cu fundamentala (eroare de octava).
double detecteazaFundamentala(double fTinta, double &magVarf) {
  double binHz = (double)SAMPLE_RATE / SAMPLES;        // ~7.81 Hz/bin

  // Banda de cautare: -35% .. +35% in jurul tintei (acopera o coarda dezacordata).
  int kLow  = (int)floor((fTinta * 0.65) / binHz);
  int kHigh = (int)ceil ((fTinta * 1.35) / binHz);
  if (kLow  < 2)             kLow  = 2;
  if (kHigh > SAMPLES / 2 - 1) kHigh = SAMPLES / 2 - 1;

  // HPS pe 3 armonice (3k trebuie sa ramana in spectru).
  int kHpsMax = (SAMPLES / 2 - 1) / 3;
  for (int k = 0; k < SAMPLES / 2; k++) {
    if (k <= kHpsMax)
      hps[k] = vReal[k] * vReal[2 * k] * vReal[3 * k];
    else
      hps[k] = vReal[k];   // fara HPS aici, dar in banda noastra suntem mereu < kHpsMax
  }

  // Varf in banda, pe spectrul HPS.
  int    kMax   = kLow;
  double hpsMax = 0;
  for (int k = kLow; k <= kHigh; k++)
    if (hps[k] > hpsMax) { hpsMax = hps[k]; kMax = k; }

  // Magnitudinea reala (din spectrul normal) la varful gasit — pentru prag SNR.
  magVarf = vReal[kMax];

  // Interpolare parabolica pe LOG-magnitudine (mai precisa pentru un varf
  // ferestruit decat pe magnitudine liniara). delta teoretic in [-0.5, 0.5].
  if (kMax <= 1 || kMax >= SAMPLES / 2 - 1)
    return kMax * binHz;
  double mS = log(vReal[kMax - 1] + 1e-9);
  double mC = log(vReal[kMax]     + 1e-9);
  double mD = log(vReal[kMax + 1] + 1e-9);
  double numitor = (mS - 2.0 * mC + mD);
  double delta   = (numitor != 0.0) ? 0.5 * (mS - mD) / numitor : 0.0;
  if (delta >  0.5) delta =  0.5;       // protectie numerica
  if (delta < -0.5) delta = -0.5;
  return (kMax + delta) * binHz;
}

double calculeazaCenti(double frecvMasurata, double frecvTinta) {
  return 1200.0 * log2(frecvMasurata / frecvTinta);
}

// ─────────────────────── Rotire motor ───────────────────────
// Setam DIR, asteptam setup, apoi dam pasii. Servim webserver-ul periodic
// ca interfata sa nu "inghete" in timpul unei corectii lungi.
void rotesteMotor(int pasi, bool directie) {
  if (pasi <= 0) return;
  abortMiscare = false;                // resetam flagul la inceputul unei miscari noi
  enableDriver(true);

  bool dir = INVERT_DIR ? !directie : directie;
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);     // DIR stabil INAINTE de primul puls

  for (int i = 0; i < pasi; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);

    if ((i % STEP_BURST_SERVICE) == 0) {
      server.handleClient();           // mentine UI-ul responsiv in timpul rotirii
      yield();
      if (abortMiscare) break;         // STOP apasat -> oprim miscarea imediat
    }
  }
  delayMicroseconds(DIR_SETUP_US);     // hold DIR dupa ultimul puls
}

// ─────────────────────── Achizitie + FFT ────────────────────
bool achizitieFFT(double &frecvFiltrata) {
  server.handleClient();

  unsigned long startTime = micros();
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (double)analogRead(MIC_PIN) - offsetDC;
    vImag[i] = 0.0;
    while (micros() - startTime < (unsigned long)(i + 1) * SAMPLE_PERIOD_US) { /* wait */ }
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  double fTinta = frecventeTinta[coardaSelectata];
  double magVarf;
  double frecvBruta = detecteazaFundamentala(fTinta, magVarf);

  // Validare: frecventa in banda + semnal suficient de puternic.
  // Praguri scalate pt SAMPLES=4096 (magnitudini mai mari). Regleaza dupa
  // valorile MAG din Serial: MAG_SEMNAL = corzi ciupite, sub MAG_LINISTE = liniste.
  const double MAG_SEMNAL  = 800.0;   // prag coarda ciupita (regleaza dupa MAG din Serial)
  const double MAG_LINISTE = 300.0;
  double fereastra   = fTinta * 0.35;
  bool   frecvValida = frecvBruta > fTinta - fereastra &&
                       frecvBruta < fTinta + fereastra &&
                       frecvBruta > 70.0 &&
                       magVarf    > MAG_SEMNAL;

  if (frecvValida) {
    double frecvMediana = filtruMedian(frecvBruta);
    double frecvIIR     = filtruIIR(frecvMediana);
    frecvFiltrata       = filtruKalman(frecvIIR);

    rawCurenta = frecvBruta;     // valorile se logheaza structurat din loop()
    magCurenta = magVarf;
    return true;
  }

  // Liniste — resetam filtrele ca sa nu ramana cu valori vechi.
  if (magVarf < MAG_LINISTE) {
    kalmanInitializat = false;
    stabilCount = 0;
    acordatCount = 0;
    resetMedian();
    resetIIR();
  }
  return false;
}

// ─────────────────────── Corectie ───────────────────────────
// Control proportional cu SUB-corectie deliberata: comand mai putini pasi decat
// ar trebui teoretic, ca sa NU depasesc tinta. Bucla remasoara si itereaza ->
// converge fin, fara oscilatii. eroare>0 = prea inalt (slabeste); <0 = strange.
void corectie(double centi) {
  if (fabs(centi) > EROARE_MAX_CENTI) {
    statusCurent = "EROARE: depasit limita siguranta";
    sistemActiv  = false;
    enableDriver(false);
    return;
  }

  int pasi = (int)lround(fabs(centi) * STEPS_PER_CENT);
  if (pasi < PASI_MIN) pasi = PASI_MIN;
  if (pasi > PASI_MAX) pasi = PASI_MAX;

  bool strange = (centi < 0);                 // prea jos -> strange
  if (strange != ultimaDirStrange) {          // schimbare de sens -> compensare joc
    pasi += BACKLASH_PASI;
    ultimaDirStrange = strange;
  }

  pasiTotal += strange ? pasi : -pasi;
  Serial.print("C,");   Serial.print(millis());   Serial.print(",");
  Serial.print(centi, 2);                          Serial.print(",");
  Serial.print(pasi);                              Serial.print(",");
  Serial.print(strange ? "STRANGE" : "SLABESTE");  Serial.print(",");
  Serial.println(pasiTotal);

  rotesteMotor(pasi, strange);
}

// ─────────────────────── Reset stare ────────────────────────
void resetSistem() {
  kalmanInitializat = false;
  stabilCount  = 0;
  acordatCount = 0;
  kalPrev      = 0;
  frecvCurenta = 0.0;
  eroareCurenta= 0.0;
  statusCurent = "Asteapta...";
  resetStabil();
  ultimaMiscare = 0;
  resetMedian();
  resetIIR();
}

// ─────────────────────── Interfata web ──────────────────────
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="ro">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
  <title>Acordor Automat</title>
  <style>
    :root{
      --bg1:#0f0c29; --bg2:#16213e; --card:rgba(255,255,255,.05);
      --line:rgba(255,255,255,.12); --txt:#eef2ff; --muted:#9aa3c0;
      --accent:#e94560; --ok:#28d17c; --near:#f5b942; --far:#e94560; --blue:#5ad1e6;
    }
    *{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
    body{
      font-family:-apple-system,Segoe UI,Roboto,Arial,sans-serif;
      color:var(--txt);min-height:100vh;padding:18px 16px 32px;
      background:radial-gradient(1200px 600px at 50% -10%,#24243e 0,var(--bg1) 55%),
                 linear-gradient(160deg,var(--bg1),var(--bg2));
      background-attachment:fixed;
    }
    .wrap{max-width:520px;margin:0 auto}
    header{text-align:center;margin-bottom:18px}
    header h1{font-size:26px;font-weight:800;letter-spacing:.3px}
    header h1 span{color:var(--accent)}
    header p{color:var(--muted);font-size:14px;margin-top:4px}
    .card{
      background:var(--card);border:1px solid var(--line);border-radius:18px;
      padding:20px;margin-bottom:16px;backdrop-filter:blur(8px);
      box-shadow:0 10px 30px rgba(0,0,0,.35);
    }
    .label{color:var(--muted);font-size:13px;font-weight:600;text-transform:uppercase;
      letter-spacing:1px;margin-bottom:14px;text-align:center}

    /* ── Readout ── */
    .readout{text-align:center;margin-bottom:6px}
    .hz{font-size:60px;font-weight:800;line-height:1;letter-spacing:-1px}
    .hz small{font-size:22px;color:var(--muted);font-weight:600;margin-left:6px}
    .target{color:var(--blue);font-size:16px;font-weight:600;margin-top:8px}
    .cents{font-size:18px;font-weight:700;margin-top:4px;color:var(--muted)}

    /* ── Ac cromatic (needle gauge) ── */
    .gauge{position:relative;height:84px;margin:18px 4px 6px}
    .scale{position:absolute;top:34px;left:0;right:0;height:14px;border-radius:8px;
      background:linear-gradient(90deg,rgba(233,69,96,.35),rgba(245,185,66,.25) 35%,
        rgba(40,209,124,.45) 50%,rgba(245,185,66,.25) 65%,rgba(233,69,96,.35));
      border:1px solid var(--line)}
    .center{position:absolute;top:24px;left:50%;width:2px;height:34px;
      transform:translateX(-50%);background:rgba(255,255,255,.55)}
    .ticks{position:absolute;top:54px;left:0;right:0;display:flex;justify-content:space-between;
      color:var(--muted);font-size:11px;font-weight:600}
    .needle{position:absolute;top:18px;left:50%;width:6px;height:46px;border-radius:4px;
      transform:translateX(-50%);background:var(--blue);transition:left .25s ease,background .25s;
      box-shadow:0 0 14px var(--blue)}
    .hint{display:flex;justify-content:space-between;color:var(--muted);font-size:12px;
      font-weight:600;margin-top:2px}
    .gauge.g-ok .needle{background:var(--ok);box-shadow:0 0 18px var(--ok)}
    .gauge.g-near .needle{background:var(--near);box-shadow:0 0 16px var(--near)}
    .gauge.g-far .needle{background:var(--far);box-shadow:0 0 16px var(--far)}
    .gauge.g-idle .needle{background:var(--muted);box-shadow:none}

    /* ── Status pill ── */
    .status{display:block;text-align:center;font-size:18px;font-weight:800;
      padding:12px;border-radius:12px;margin-top:14px;background:rgba(255,255,255,.06);
      border:1px solid var(--line)}
    .status.s-ok{color:#062;background:var(--ok);border-color:transparent}
    .status.s-high{color:#fff;background:var(--far);border-color:transparent}
    .status.s-low{color:#3a2a00;background:var(--near);border-color:transparent}
    .status.s-idle{color:var(--muted)}

    /* ── Selector corzi ── */
    .corzi{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
    .str{padding:18px 8px;border:1px solid var(--line);border-radius:14px;
      background:rgba(255,255,255,.04);color:var(--txt);font-size:20px;font-weight:800;
      cursor:pointer;transition:.18s;min-height:74px}
    .str small{display:block;font-size:12px;color:var(--muted);font-weight:600;margin-top:4px}
    .str:active{transform:scale(.97)}
    .str.on{background:var(--accent);border-color:var(--accent);color:#fff;
      box-shadow:0 6px 18px rgba(233,69,96,.45)}
    .str.on small{color:rgba(255,255,255,.85)}

    /* ── Butoane control ── */
    .btn{width:100%;padding:20px;border:none;border-radius:14px;font-size:20px;
      font-weight:800;cursor:pointer;transition:.18s;letter-spacing:.3px}
    .btn:active{transform:scale(.98)}
    .btn-start{background:linear-gradient(135deg,#28d17c,#1ba85f);color:#04261a;
      box-shadow:0 8px 22px rgba(40,209,124,.35);margin-bottom:12px}
    .btn-stop{background:rgba(233,69,96,.16);color:var(--accent);border:1px solid var(--accent)}

    /* ── Avansat (test motor) ── */
    details{margin-top:4px}
    details>summary{list-style:none;cursor:pointer;color:var(--muted);font-size:14px;
      font-weight:600;text-align:center;padding:10px}
    details>summary::-webkit-details-marker{display:none}
    .test-row{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:10px}
    .btn-test{padding:16px;border:1px solid var(--line);border-radius:12px;
      background:rgba(255,255,255,.04);color:var(--blue);font-size:15px;font-weight:700;cursor:pointer}
    .btn-test:active{transform:scale(.97)}
  </style>
</head>
<body>
  <div class="wrap">
    <header>
      <h1>🎸 Acordor <span>Automat</span></h1>
      <p>Selecteaza coarda, apasa START si ciupeste</p>
    </header>

    <div class="card">
      <div class="readout">
        <div class="hz"><span id="hz">––</span><small>Hz</small></div>
        <div class="target" id="target">A2 · 110.00 Hz</div>
        <div class="cents" id="cents">–</div>
      </div>

      <div class="gauge g-idle" id="gauge">
        <div class="scale"></div>
        <div class="center"></div>
        <div class="needle" id="needle"></div>
        <div class="ticks"><span>-50</span><span>-25</span><span>0</span><span>+25</span><span>+50</span></div>
      </div>
      <div class="hint"><span>&#9837; prea jos</span><span>prea sus &#9839;</span></div>

      <div class="status s-idle" id="status" aria-live="polite">Asteapta...</div>
    </div>

    <div class="card">
      <div class="label">Coarda</div>
      <div class="corzi" id="corzi">
        <button class="str" onclick="pick(0)">E2<small>82.41 Hz</small></button>
        <button class="str" onclick="pick(1)">A2<small>110.00 Hz</small></button>
        <button class="str" onclick="pick(2)">D3<small>146.83 Hz</small></button>
        <button class="str" onclick="pick(3)">G3<small>196.00 Hz</small></button>
        <button class="str" onclick="pick(4)">B3<small>246.94 Hz</small></button>
        <button class="str" onclick="pick(5)">E4<small>329.63 Hz</small></button>
      </div>
    </div>

    <div class="card">
      <button class="btn btn-start" onclick="start()">&#9654; START Acordare</button>
      <button class="btn btn-stop" onclick="stop()">&#9632; STOP</button>
      <details>
        <summary>Avansat · test motor</summary>
        <div class="test-row">
          <button class="btn-test" onclick="testDir(1)">Sens + (strange)</button>
          <button class="btn-test" onclick="testDir(0)">Sens − (slabeste)</button>
        </div>
      </details>
    </div>
  </div>

  <script>
    const STRINGS=[{n:'E2',f:82.41},{n:'A2',f:110.00},{n:'D3',f:146.83},
                   {n:'G3',f:196.00},{n:'B3',f:246.94},{n:'E4',f:329.63}];
    let sel=1;
    function pick(i){ sel=i; paintStrings(); fetch('/coarda?idx='+i); }
    function start(){ fetch('/start'); }
    function stop(){ fetch('/stop'); }
    function testDir(d){ fetch('/test?dir='+d); }

    function paintStrings(){
      document.querySelectorAll('.str').forEach((b,i)=>b.classList.toggle('on', i===sel));
      const s=STRINGS[sel];
      document.getElementById('target').textContent = s.n + ' · ' + s.f.toFixed(2) + ' Hz';
    }

    async function poll(){
      try{
        const r = await fetch('/status'); const d = await r.json();
        if (typeof d.coarda === 'number') sel = d.coarda;
        paintStrings();
        const hasSig = d.frecv > 0;
        document.getElementById('hz').textContent = hasSig ? d.frecv.toFixed(2) : '––';
        const cents = d.eroare;
        document.getElementById('cents').textContent =
          hasSig ? ((cents>0?'+':'') + cents.toFixed(1) + ' centi') : '–';

        const c = Math.max(-50, Math.min(50, cents));
        document.getElementById('needle').style.left = (hasSig ? (50 + c) : 50) + '%';

        const g = document.getElementById('gauge');
        g.className = 'gauge ' + (!hasSig ? 'g-idle'
          : Math.abs(cents)<=5 ? 'g-ok'
          : Math.abs(cents)<=20 ? 'g-near' : 'g-far');

        const st = document.getElementById('status');
        st.textContent = d.status;
        st.className = 'status ' + (
          d.status.includes('ACORDATA') ? 's-ok'
          : d.status.includes('INALTA') ? 's-high'
          : d.status.includes('JOASA')  ? 's-low' : 's-idle');
      }catch(e){}
    }
    setInterval(poll, 200);
    paintStrings();
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleStatus() {
  String json = "{";
  json += "\"frecv\":"    + String(frecvCurenta, 2) + ",";
  json += "\"eroare\":"   + String(eroareCurenta, 1) + ",";
  json += "\"status\":\"" + statusCurent + "\",";
  json += "\"coarda\":"   + String(coardaSelectata);
  json += "}";
  server.send(200, "application/json", json);
}

void handleCoarda() {
  if (server.hasArg("idx")) {
    int idx = server.arg("idx").toInt();
    if (idx >= 0 && idx <= 5) { coardaSelectata = idx; resetSistem(); }
  }
  server.send(200, "text/plain", "ok");
}

void handleStart() {
  sistemActiv  = true;
  resetSistem();
  enableDriver(true);
  pasiTotal = 0;
  Serial.println("=== START ===");
  Serial.println("LOG  F=ms,coarda,target,raw,filtrat,centi,mag | C=ms,centi,pasi,dir,total | T=ms,centi,total");
  statusCurent = "Ciupeste coarda...";
  server.send(200, "text/plain", "ok");
}

void handleStop() {
  sistemActiv  = false;
  abortMiscare = true;             // intrerupe orice burst de pasi in curs
  statusCurent = "Oprit";
  enableDriver(false);
  server.send(200, "text/plain", "ok");
}

// Test manual al sensului motorului: roteste un numar fix de pasi.
void handleTest() {
  int dir = 1;
  if (server.hasArg("dir")) dir = server.arg("dir").toInt();
  server.send(200, "text/plain", "ok");      // raspunde inainte de blocare
  statusCurent = (dir ? "TEST: sens + (strange)" : "TEST: sens - (slabeste)");
  rotesteMotor(200, dir != 0);               // 200 pasi de test
  statusCurent = "Test terminat";
  enableDriver(false);
}

// ─────────────────────── Setup ──────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(EN_PIN,   OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);
  digitalWrite(EN_PIN, EN_ACTIV_LOW ? LOW : HIGH);   // driver ACTIV de la pornire
  driverActiv = true;

  // ── Config ADC explicit (ESP32-S3) ──
  analogReadResolution(12);              // 0..4095
  analogSetPinAttenuation(MIC_PIN, ADC_11db);   // ~0..3.3V

  // ── Calibrare offset DC microfon (liniste la pornire) ──
  calibreazaOffsetDC();

  // ── WiFi ──
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                   // FARA modem-sleep -> serverul web raspunde prompt
  if (USE_STATIC_IP) {                    // adresa fixa
    if (!WiFi.config(ip_static, ip_gateway, ip_subnet, ip_dns))
      Serial.println("Config IP static esuat");
  }
  WiFi.begin(ssid, password);
  Serial.print("Conectare WiFi");
  int incercari = 0;
  while (WiFi.status() != WL_CONNECTED && incercari < 40) {
    delay(500); Serial.print("."); incercari++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi neconectat — verifica SSID/parola (doar 2.4GHz).");
  }

  server.on("/",       handleRoot);
  server.on("/status", handleStatus);
  server.on("/coarda", handleCoarda);
  server.on("/start",  handleStart);
  server.on("/stop",   handleStop);
  server.on("/test",   handleTest);
  server.begin();
  Serial.println("Server pornit");
}

// ─────────────────────── Loop ───────────────────────────────
// Disciplina: MASOR (mai multe cadre, pana cand citirea e stabila) -> DECID ->
// MISC -> ASTEPT (settle) -> MASOR din nou. Asa elimin oscilatia, jitter-ul si
// lock-ul fals: nu acordez si nu declar "ACORDATA" decat pe o citire stabila.
void loop() {
  server.handleClient();

  if (!sistemActiv) { delay(20); return; }

  // Dupa o miscare, las coarda + mecanica sa se aseze inainte de a remasura.
  if (millis() - ultimaMiscare < SETTLE_MS) {
    statusCurent = "Stabilizare...";
    delay(5);
    return;
  }

  double f = 0;
  if (!achizitieFFT(f)) {
    statusCurent = "Ciupeste coarda...";
    resetStabil();                 // semnal pierdut -> reincepem masurarea stabila
    return;
  }

  double centi = calculeazaCenti(f, frecventeTinta[coardaSelectata]);
  frecvCurenta  = f;
  eroareCurenta = centi;
  pushStabil(centi);

  // Log structurat per cadru (CSV): F,ms,coarda,target,raw,filtrat,centi,mag
  Serial.print("F,");  Serial.print(millis());                       Serial.print(",");
  Serial.print(coardaSelectata);                                     Serial.print(",");
  Serial.print(frecventeTinta[coardaSelectata], 2);                  Serial.print(",");
  Serial.print(rawCurenta, 2);                                       Serial.print(",");
  Serial.print(f, 2);                                                Serial.print(",");
  Serial.print(centi, 2);                                            Serial.print(",");
  Serial.println(magCurenta, 0);

  // ── ACORDAT? — declaram doar pe o citire STABILA (anti lock-fals) ──
  if (stabilGata()) {
    double centiS = medianaStabil();
    eroareCurenta = centiS;
    if (fabs(centiS) <= TOL_CENTI) {
      statusCurent = "ACORDATA";
      sistemActiv  = false;
      enableDriver(false);          // TINE_MEREU_TENSIUNE=true il pastreaza energizat
      Serial.print("T,"); Serial.print(millis()); Serial.print(",");
      Serial.print(centiS, 2); Serial.print(","); Serial.println(pasiTotal);
      return;
    }
  }

  // ── MISCARE? — destul sa avem 2 cadre consecutive de ACELASI SEMN.
  // Asa motorul corecteaza fiabil chiar daca citirea nu e perfect stabila
  // (corzile groase/care se sting repede), iar sub-corectia (STEPS_PER_CENT mic)
  // impiedica depasirea tintei. Doar pe un singur cadru-glitch NU miscam.
  if (fabs(centi) > TOL_CENTI && stabilCnt >= 2 && acelasiSemn()) {
    statusCurent = (centi > 0) ? "PREA INALTA - slabeste"
                               : "PREA JOASA - strange";
    corectie(centi);
    resetStabil();
    ultimaMiscare = millis();
    return;
  }

  statusCurent = "Masurare...";
}
