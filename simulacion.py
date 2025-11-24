import sys
import time
import numpy as np
from collections import deque

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

#parametros
m = 1150.0 #kg
b = 50.0 #coeficiente de rozamiento
k_throttle = 3500.0 #traccion / angulo
tau_th = 0.4
g = 9.81

Kp = 0.45
Ki = 0.22

max_aceleracion = 500.0
max_cambio_aceleracion = 0.8
min_valvula = 0.0
max_valvula = 1.0

dt = 0.05
window_len = 200

Vref_kmh = 60.0
v_kmh = 40.0
theta = 0.0
theta_anterior = 0.0
integral = 0.0
prev_error = 0.0

viento = {"mag": 0.0, "unidad": "m/s"}
viento_estado = False

rho = 1.225
cda = 0.66

pendiente = 0.0
pendiente_estado = False

#limite
fuerza_vencer_max = 3500.0 #N

times = deque(maxlen=window_len)
vels = deque(maxlen=window_len)
refs = deque(maxlen=window_len)
error_log = deque(maxlen=window_len)
controller_log = deque(maxlen=window_len)
pwm_log = deque(maxlen=window_len)
perturbacion_log = deque(maxlen=window_len)

theta_real_log = deque(maxlen=window_len)
p_log = deque(maxlen=window_len)
i_log = deque(maxlen=window_len)
pi_log = deque(maxlen=window_len)

t_sim = 0.0

def calcular_controlador(dt, err, integral):
    P = Kp * err
    integral_candidato = integral + err * dt
    I = Ki * integral_candidato
    suma_pi = P + I
    return I, P, integral_candidato, suma_pi

def aplicar_control(v_kmh, theta, theta_anterior, integral, prev_error, Vref_kmh, dt):
    err = (Vref_kmh - v_kmh) / 3.6
    I, P, integral_candidato, suma_pi = calcular_controlador(dt, err, integral)

    v = v_kmh / 3.6
    f_resistencia = b * v

    if viento["unidad"] == "m/s":
        v_viento = viento["mag"]
    else:
        v_viento = viento["mag"] / 3.6

    v_rel = v

    if viento_estado:
        v_rel = v - v_viento

    f_aero = 0.5 * rho * cda * v_rel * abs(v_rel)

    pendiente_frac = pendiente / 100.0
    grado_pendiente = np.arctan(pendiente_frac)
    f_pendiente = m * g * np.sin(grado_pendiente) if pendiente_estado else 0.0

    pwm = np.clip(suma_pi, min_valvula, max_valvula)

    if not((suma_pi > max_valvula and err > 0) or (suma_pi < min_valvula and err < 0)):
        integral = integral_candidato
        I = Ki * integral

    max_delta = max_cambio_aceleracion * dt
    theta_ref = theta_anterior + np.clip(pwm - theta_anterior, -max_delta, max_delta)
    theta_anterior = theta_ref

    dtheta_dt = (theta_ref - theta) / tau_th
    theta += dtheta_dt * dt
    theta = float(np.clip(theta, min_valvula, max_valvula))

    f_trac = k_throttle * theta
    a = (f_trac - f_resistencia - f_aero - f_pendiente) / m
    v += a * dt
    v = max(0, v)

    v_kmh = v * 3.6

    diag = {
        "P": P, "I": I,
        "suma_pi": suma_pi,
        "pwm": pwm,
        "error_velocidad": err
    }

    return v_kmh, theta, theta_anterior, integral, err, diag

def calcular_fuerza_vencer(v_kmh_local, viento_mag_local, pendiente_pct_local):
    v = v_kmh_local / 3.6
    f_res = b * v

    v_w = float(viento_mag_local)
    v_rel = v - v_w
    f_aero = 0.5 * rho * cda * v_rel * abs(v_rel)

    frac = pendiente_pct_local / 100.0
    ang = np.arctan(frac)
    f_pend = m * g * np.sin(ang)

    return f_res + f_aero + f_pend

class Ventana(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulador Control Velocidad")

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        pg.setConfigOptions(antialias=True)

        self.alert_label = QtWidgets.QLabel("")
        self.alert_label.setAlignment(QtCore.Qt.AlignCenter)
        self.alert_clear_timer = None
        main_layout.addWidget(self.alert_label)

        self.plot_speed = pg.PlotWidget(title="Velocidad (km/h)")
        self.line_ref = self.plot_speed.plot(pen=pg.mkPen('w',style=QtCore.Qt.DashLine), name="Vref")
        self.line_feedback = self.plot_speed.plot(pen='y', name="Vel real")
        self.plot_speed.addLegend(offset=(5, 5))

        self.plot_error = pg.PlotWidget(title="Señal de error (km/h)")
        self.line_error = self.plot_error.plot(pen='r')

        self.plot_contoller = pg.PlotWidget(title="Salida del controlador")
        self.line_controller = self.plot_contoller.plot(pen='g', name='suma_pi')
        self.line_pwm = self.plot_contoller.plot(pen=pg.mkPen(style=QtCore.Qt.DotLine), name="pwm")
        self.plot_contoller.addLegend(offset=(5, 5))

        self.plot_perturbacion = pg.PlotWidget(title="Perturbaciones sumadas (N)")
        self.line_perturbacion = self.plot_perturbacion.plot(pen='m')

        main_layout.addWidget(self.plot_speed)
        main_layout.addWidget(self.plot_error)
        main_layout.addWidget(self.plot_contoller)
        main_layout.addWidget(self.plot_perturbacion)

        controls = QtWidgets.QHBoxLayout()

        col_left = QtWidgets.QVBoxLayout()
        estadisticas_left_widget = QtWidgets.QWidget()
        estadisticas_left_layout = QtWidgets.QVBoxLayout()
        estadisticas_left_widget.setLayout(estadisticas_left_layout)
        estadisticas_left_layout.setContentsMargins(2, 2, 2, 2)
        estadisticas_left_layout.addWidget(QtWidgets.QLabel("<b>Valores reales</b>"))
        self.lbl_vref_small = QtWidgets.QLabel(f"Vref: {Vref_kmh:.1f} km/h")
        self.lbl_vreal_small = QtWidgets.QLabel(f"Vel real: {v_kmh:.1f} km/h")
        self.lbl_perturbacion_small = QtWidgets.QLabel(f"Perturbación: {0.0:.1f} N")
        self.lbl_error_small = QtWidgets.QLabel(f"Error: {0.0:.2f} km/h")
        estadisticas_left_layout.addWidget(self.lbl_vref_small)
        estadisticas_left_layout.addWidget(self.lbl_vreal_small)
        estadisticas_left_layout.addWidget(self.lbl_perturbacion_small)
        estadisticas_left_layout.addWidget(self.lbl_error_small)
        col_left.addWidget(estadisticas_left_widget)

        viento_layout = QtWidgets.QVBoxLayout()
        self.lbl_viento = QtWidgets.QLabel("Viento (m/s): 0")
        self.lbl_viento.setAlignment(QtCore.Qt.AlignCenter)
        viento_layout.addWidget(self.lbl_viento)
        self.viento_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.viento_slider.setRange(-27, 27)
        self.viento_slider.setValue(0)
        self.viento_slider.valueChanged.connect(self.cambio_viento_slider)
        viento_layout.addWidget(self.viento_slider)
        col_left.addLayout(viento_layout)
        controls.addLayout(col_left)

        col_right = QtWidgets.QVBoxLayout()
        estadisticas_right_widget = QtWidgets.QWidget()
        estadisticas_right_layout = QtWidgets.QVBoxLayout()
        estadisticas_right_widget.setLayout(estadisticas_right_layout)
        estadisticas_right_layout.setContentsMargins(2,2,2,2)
        estadisticas_right_layout.addWidget(QtWidgets.QLabel("<b>Control / válvula</b>"))
        self.lbl_theta_small = QtWidgets.QLabel(f"Theta: {theta:.3f} (frac) / {theta*100:.1f}%")
        self.lbl_P_small = QtWidgets.QLabel(f"P: {0.0:.4f} m/s")
        self.lbl_I_small = QtWidgets.QLabel(f"I: {0.0:.4f} m/s")
        self.lbl_suma_pi_small = QtWidgets.QLabel(f"Suma PI: {0.0:.4f}")
        self.lbl_pwm_small = QtWidgets.QLabel(f"PWM: {0.0:.4f}")
        estadisticas_right_layout.addWidget(self.lbl_theta_small)
        estadisticas_right_layout.addWidget(self.lbl_P_small)
        estadisticas_right_layout.addWidget(self.lbl_I_small)
        estadisticas_right_layout.addWidget(self.lbl_suma_pi_small)
        estadisticas_right_layout.addWidget(self.lbl_pwm_small)
        col_right.addWidget(estadisticas_right_widget)

        pendiente_layout = QtWidgets.QVBoxLayout()
        self.lbl_pendiente = QtWidgets.QLabel("Pendiente (%): 0")
        self.lbl_pendiente.setAlignment(QtCore.Qt.AlignCenter)
        pendiente_layout.addWidget(self.lbl_pendiente)
        self.pendiente_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.pendiente_slider.setRange(-3, 15)
        self.pendiente_slider.setValue(0)
        self.pendiente_slider.valueChanged.connect(self.cambio_pendiente_slider)
        pendiente_layout.addWidget(self.pendiente_slider)
        col_right.addLayout(pendiente_layout)

        controls.addLayout(col_right)

        btn_layout = QtWidgets.QVBoxLayout()
        self.chk_live = QtWidgets.QCheckBox("Actualizar en vivo")
        btn_apply = QtWidgets.QPushButton("Aplicar perturbaciones")
        btn_reset = QtWidgets.QPushButton("Reset perturbaciones")
        btn_rafaga = QtWidgets.QPushButton("Ráfaga (3s)")
        btn_cerrar_alerta = QtWidgets.QPushButton("Cerrar alerta")

        btn_apply.clicked.connect(self.aplicar_perturbaciones)
        btn_reset.clicked.connect(self.reset_perturbaciones)
        btn_rafaga.clicked.connect(self.rafaga_perturbacion)
        btn_cerrar_alerta.clicked.connect(self.clear_alert)

        btn_layout.addWidget(self.chk_live)
        btn_layout.addWidget(btn_apply)
        btn_layout.addWidget(btn_rafaga)
        btn_layout.addWidget(btn_reset)
        btn_layout.addWidget(btn_cerrar_alerta)

        controls.addLayout(btn_layout)
        main_layout.addLayout(controls)

        self.viento_temp = 0.0
        self.pendiente_temp = 0.0
        self.rafaga_duracion_ms = 3000
        self._rafaga_backup = None

        self.viento_antes_aplicar = 0.0
        self.pendiente_antes_aplicar = 0.0

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_sim)
        self.timer.start(int(dt * 1000))

        self.show()

    def mostrar_alerta(self, msg: str, level: str = "error", timeout_ms: int = 0):
        if level == "error":
            style = "background:#ffcccc; color:#700000; padding:6px; border:1px solid #700000; font-weight:bold;"
        elif level == "warning":
            style = "background:#fff0cc; color:#6a4a00; padding:6px; border:1px solid #6a4a00; font-weight:bold;"
        else:
            style = "background:#f0f0f0; color:#222; padding:6px; border:1px solid #ccc;"

        self.alert_label.setStyleSheet(style)
        self.alert_label.setText(msg)

        if self.alert_clear_timer is not None and self.alert_clear_timer.isActive():
            self.alert_clear_timer.stop()

        if timeout_ms and timeout_ms > 0:
            self.alert_clear_timer = QtCore.QTimer(self)
            self.alert_clear_timer.setSingleShot(True)
            self.alert_clear_timer.timeout.connect(self.clear_alert)
            self.alert_clear_timer.start(timeout_ms)

    def clear_alert(self):
        self.alert_label.setText("")
        self.alert_label.setStyleSheet("")
        if self.alert_clear_timer is not None and self.alert_clear_timer.isActive():
            self.alert_clear_timer.stop()
        self.alert_clear_timer = None

    def _update_small_stats(self, diag=None, fuerza_actual=None):
        self.lbl_vref_small.setText(f"Vref: {Vref_kmh:.1f} km/h")
        self.lbl_vreal_small.setText(f"Vel real: {v_kmh:.1f} km/h")

        if fuerza_actual is not None:
            self.lbl_perturbacion_small.setText(f"Perturbación: {fuerza_actual:.1f} N")
        else:
            fcalc = calcular_fuerza_vencer(v_kmh, viento["mag"], pendiente)
            self.lbl_perturbacion_small.setText(f"Perturbación: {fcalc:.1f} N")

        if diag is not None:
            err_m_s = diag.get("error_velocidad", 0.0)
            self.lbl_error_small.setText(f"Error: {err_m_s*3.6:.2f} km/h")
            P = diag.get("P", 0.0)
            I = diag.get("I", 0.0)
            suma_pi = diag.get("suma_pi", 0.0)
            pwm = diag.get("pwm", 0.0)
            self.lbl_P_small.setText(f"P: {P:0.4f} m/s")
            self.lbl_I_small.setText(f"I: {I:0.4f} m/s")
            self.lbl_suma_pi_small.setText(f"Suma PI: {suma_pi:.4f}")
            self.lbl_pwm_small.setText(f"PWM: {pwm:.4f}")
        else:
            self.lbl_error_small.setText("Error: 0.00 km/h")
            self.lbl_P_small.setText("P: 0.0000 m/s")
            self.lbl_I_small.setText("I: 0.0000 m/s")
            self.lbl_suma_pi_small.setText("Suma PI: 0.0000")
            self.lbl_pwm_small.setText("PWM: 0.0000")

        self.lbl_theta_small.setText(f"Theta: {theta:.3f} (frac) / {theta*100:.1f}%")

    def cambio_viento_slider(self, value):
        self.viento_temp = float(value)
        self.lbl_viento.setText(f"Viento (m/s): {self.viento_temp}")
        if self.chk_live.isChecked():
            self._intentar_aplicar_viento(self.viento_temp)

    def cambio_pendiente_slider(self, value):
        self.pendiente_temp = float(value)
        self.lbl_pendiente.setText(f"Pendiente (%): {self.pendiente_temp}")
        if self.chk_live.isChecked():
            self._intentar_aplicar_pendiente(self.pendiente_temp)

    def aplicar_perturbaciones(self):
        fuerza = calcular_fuerza_vencer(v_kmh, self.viento_temp, self.pendiente_temp)

        if fuerza > fuerza_vencer_max:
            self.mostrar_alerta(
                f"FUERA DE CONTEXTO: fuerza a vencer = {fuerza:.1f} N (> {fuerza_vencer_max:.1f} N). "
                "Estabilidad NO garantizada.",
                level="error",
                timeout_ms=0
            )
        elif fuerza < 0.0:
            self.mostrar_alerta(
                f"FUERA DE CONTEXTO: fuerza a vencer = {fuerza:.1f} N (< 0). Estabilidad NO garantizada.",
                level="warning",
                timeout_ms=0
            )
        else:
            self.mostrar_alerta(f"Perturbación aplicada. Fuerza a vencer = {fuerza:.1f} N", level="info", timeout_ms=3000)

        self._aplicar_viento_ahora(self.viento_temp)
        self._aplicar_pendiente_ahora(self.pendiente_temp)
        self.viento_antes_aplicar = float(self.viento_temp)
        self.pendiente_antes_aplicar = float(self.pendiente_temp)

        self._update_small_stats(fuerza_actual=fuerza)
        print(f"Aplicadas perturbaciones: viento={viento['mag']} m/s pendiente={pendiente} % (Fuerza {fuerza:.1f} N)")

    def reset_perturbaciones(self):
        self.viento_slider.setValue(0)
        self.pendiente_slider.setValue(0)
        self._aplicar_viento_ahora(0.0)
        self._aplicar_pendiente_ahora(0.0)
        self.viento_antes_aplicar = 0.0
        self.pendiente_antes_aplicar = 0.0
        self.clear_alert()

        self._update_small_stats()
        print("Perturbaciones reseteadas a 0.")

    def rafaga_perturbacion(self):
        global viento, viento_estado
        valor_rafaga = float(self.viento_temp)
        if valor_rafaga == 0.0:
            self.mostrar_alerta("Ráfaga: el valor del slider es 0, no se aplica ráfaga.", level="info",timeout_ms=3000)
            return

        fuerza = calcular_fuerza_vencer(v_kmh, valor_rafaga, pendiente)

        if fuerza > fuerza_vencer_max:
            self.mostrar_alerta(
                f"FUERA DE CONTEXTO: ráfaga no recomendada. Fuerza a vencer = {fuerza:.1f} N (> {fuerza_vencer_max:.1f} N.",
                level="error",
                timeout_ms=0
            )
        elif fuerza < 0.0:
            self.mostrar_alerta(
                f"FUERA DE CONTEXTO: ráfaga fuera de contexto (fuerza = {fuerza:.1f} N). Se aplicará igualmente.",
                level="warning",
                timeout_ms=0
            )
        else:
            self.mostrar_alerta(f"Ráfaga aplicada. Fuerza a vencer = {fuerza:.1f} N.", level="info", timeout_ms=3000)

        self._rafaga_backup = (viento["mag"], viento_estado)
        viento["mag"] = valor_rafaga
        viento_estado = True

        self._update_small_stats(fuerza_actual=fuerza)
        print(f"Ráfaga aplicada: viento={valor_rafaga} m/s por {self.rafaga_duracion_ms/1000:.1f}s (Fuerza {fuerza:.1f} N)")
        QtCore.QTimer.singleShot(self.rafaga_duracion_ms,self._rafaga_fin)

    def _rafaga_fin(self):
        global viento, viento_estado
        if self._rafaga_backup is not None:
            viento["mag"], viento_estado = self._rafaga_backup

            self._update_small_stats()
            print(f"Ráfaga terminada. Viento restaurado a {viento['mag']} m/s")
            self._rafaga_backup = None

    def _aplicar_viento_ahora(self, mag):
        global viento, viento_estado
        viento["mag"] = float(mag)
        viento_estado = True if mag != 0.0 else False
        self._update_small_stats()

    def _aplicar_pendiente_ahora(self, pct):
        global pendiente, pendiente_estado
        pendiente = float(pct)
        pendiente_estado = True if pct != 0.0 else False
        self._update_small_stats()

    def _intentar_aplicar_viento(self, mag):
        fuerza = calcular_fuerza_vencer(v_kmh, mag, pendiente)
        if fuerza > fuerza_vencer_max:
            self.mostrar_alerta(
                f"FUERA DE CONTEXTO (live): fuerza a vencer = {fuerza:.1f} N (> {fuerza_vencer_max:.1f} N)."
                "Estabilidad NO garantizada.",
                level="error",
                timeout_ms=0
            )
        elif fuerza < 0.0:
            self.mostrar_alerta(
                f"FUERA de CONTEXTO (live): fuerza a vencer = {fuerza:.1f} N (<0). Estabilidad NO garantizada.",
                level="warning",
                timeout_ms=0
            )
        else:
            self.mostrar_alerta(f"Viento aplicado (live). Fuerza a vencer = {fuerza:.1f} N.",level="info",timeout_ms=2000)

        self._aplicar_viento_ahora(mag)
        self.viento_antes_aplicar = float(mag)
        return True

    def _intentar_aplicar_pendiente(self, pct):
        fuerza = calcular_fuerza_vencer(v_kmh, viento["mag"], pct)
        if fuerza > fuerza_vencer_max:
            self.mostrar_alerta(
                f"FUERA DE CONTEXTO (live): fuerza a vencer = {fuerza:.1f} N (> {fuerza_vencer_max:.1f} N)."
                "Estabilidad NO garantizada.",
                level="error",
                timeout_ms=0
            )
        elif fuerza < 0.0:
            self.mostrar_alerta(
                f"FUERA de CONTEXTO (live): fuerza a vencer = {fuerza:.1f} N (<0). Estabilidad NO garantizada.",
                level="warning",
                timeout_ms=0
            )
        else:
            self.mostrar_alerta(f"Pendiente aplicada (live). Fuerza a vencer = {fuerza:.1f} N.",level="info",timeout_ms=2000)

        self._aplicar_pendiente_ahora(pct)
        self.pendiente_antes_aplicar = float(pct)
        return True


     #simulacion
    def update_sim(self):
        global v_kmh, theta, theta_anterior, integral, prev_error, t_sim

        v_kmh, theta, theta_anterior, integral, prev_error, diag \
            = aplicar_control(v_kmh, theta, theta_anterior, integral, prev_error, Vref_kmh, dt)

        t_sim += dt

        fuerza_actual = calcular_fuerza_vencer(v_kmh, viento["mag"], pendiente)

        times.append(t_sim)
        refs.append(Vref_kmh)
        vels.append(v_kmh)
        error_log.append(diag["error_velocidad"] * 3.6)
        controller_log.append(diag["suma_pi"])
        pwm_log.append(diag["pwm"])
        perturbacion_log.append(fuerza_actual)

        theta_real_log.append(theta*100)
        p_log.append(diag["P"])
        i_log.append(diag["I"])
        pi_log.append(diag["suma_pi"])

        self._update_small_stats(diag=diag, fuerza_actual=fuerza_actual)

        self.line_ref.setData(times, refs)
        self.line_feedback.setData(times, vels)

        self.line_error.setData(times, error_log)

        self.line_controller.setData(times, controller_log)
        self.line_pwm.setData(times, pwm_log)

        self.line_perturbacion.setData(times, perturbacion_log)

        if len(vels) > 0:
            vmin = max(min(min(vels), min(refs)) -5,0)
            vmax = max(max(vels), max(refs)) + 5
            self.plot_speed.setYRange(vmin,vmax, padding=0)

        if len(error_log) > 0:
            emin = min(error_log)
            emax = max(error_log)
            if emax - emin < 1:
                center = (emin + emax) / 2
                emin = center - 1
                emax = center + 1
            self.plot_error.setYRange(emin, emax, padding=0)

        if len(controller_log) > 0:
            cmin = min(min(controller_log), min(pwm_log))
            cmax = max(max(controller_log), max(pwm_log))
            if cmin == cmax:
                self.plot_contoller.setYRange(cmin - 0.1, cmax + 0.1, padding=0)
            else:
                self.plot_contoller.setYRange(cmin - 0.2 * abs(cmin), cmax + 0.2 * abs(cmax), padding=0)

        if len(perturbacion_log) > 0:
            pmin = min(perturbacion_log)
            pmax = max(perturbacion_log)
            if pmax == pmin:
                self.plot_perturbacion.setYRange(pmin - 10, pmax + 10)
            else:
                self.plot_perturbacion.setYRange(pmin - 0.1 * abs(pmin), pmax + 0.1 * abs(pmax))

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()

    def keyPressEvent(self, event):
        global Vref_kmh
        if event.key() == QtCore.Qt.Key_Plus or event.key() == QtCore.Qt.Key_Equal:
            Vref_kmh = min(Vref_kmh + 5, 130.0)
            print(f"Vref -> {Vref_kmh:.1f} km/h")
        elif event.key() == QtCore.Qt.Key_Minus:
            Vref_kmh = max(Vref_kmh - 5, 30.0)
            print(f"Vref -> {Vref_kmh:.1f} km/h")

app = QtWidgets.QApplication(sys.argv)
ventana = Ventana()
sys.exit(app.exec_())

