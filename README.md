# Autonomous Polyphonic Guitar Tuner (Mechatronics Bachelor Thesis)

An automated closed-loop mechatronic system designed to tune guitars by capturing multi-string acoustic signals, identifying fundamental frequencies using Fast Fourier Transform (FFT), and driving stepper motors to adjust peg tension until target frequencies are reached.

---

### Demonstration

A video recording of the working prototype is available in the repository: `demo_tuner.mp4`.

---

### System Architecture

* **Signal Processing:** Polyphonic frequency extraction based on FFT algorithms implemented on a dual-core embedded processor.
* **Control Strategy:** Closed-loop feedback system with PID correction for position and tension adjustment.
* **Embedded Software:** ESP32 running FreeRTOS tasks to separate signal sampling, motor control, and user feedback.
* **Actuation:** NEMA 17 stepper motors with silent drivers for precise angular steps.
* **Mechanical Structure:** Custom motor mounts, peg adapters, and support brackets modeled in CATIA V5 and fabricated via 3D printing.

---

### Repository Files

* `Lucrare_Licenta.pdf` — Complete Bachelor's degree thesis documentation (system modeling, calculations, electrical schematics, and experimental results).
* `Prezentare_Licenta.pdf` — Thesis defense presentation slides.
* `demo_tuner.mp4` — Video capture of the system operating on the test rig.

---

Full firmware source files and CAD models (.STEP / .STL) will be added upon cleanup.
