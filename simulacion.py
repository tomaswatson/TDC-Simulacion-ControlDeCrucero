import sys
import time
import numpy as np
from collections import deque

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QKeySequence
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

vels_medida = deque(maxlen=window_len)

t_sim = 0.0

perturbacion_estado = False
fuerza_perturbacion_actual = 0.0

num_imanes = 2
radio_rueda = 0.3
dist_entre_pulsos = (2 * np.pi * radio_rueda) / num_imanes

class SensorDRV5023:
    def __init__(self):
        self.acumulador_distancia = 0.0
        self.v_leida_ms = 0.0

    def leer_f_t(self, v_real_ms, dt):
        self.acumulador_distancia += v_real_ms * dt
        if self.acumulador_distancia >= dist_entre_pulsos:
            n_pulsos = int(self.acumulador_distancia / dist_entre_pulsos)
            self.v_leida_ms = v_real_ms
            self.acumulador_distancia -= (n_pulsos * dist_entre_pulsos)
        return self.v_leida_ms

sensor_hall = SensorDRV5023()

def calcular_controlador(dt, err, integral):
    P = Kp * err
    integral_candidato = integral + err * dt
    I = Ki * integral_candidato
    suma_pi = P + I
    return I, P, integral_candidato, suma_pi


def aplicar_control(v_kmh, theta, theta_anterior, integral, Vref_kmh, dt):
    global fuerza_perturbacion_actual, perturbacion_estado

    v_medida_kmh = sensor_hall.leer_f_t(v_kmh / 3.6, dt) * 3.6

    err = (Vref_kmh - v_medida_kmh) / 3.6
    I, P, integral_candidato, suma_pi = calcular_controlador(dt, err, integral)

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

    fuerza_resistencia = (v_kmh/3.6) * b

    f_ext_aplicada = fuerza_perturbacion_actual if perturbacion_estado else 0.0

    v = v_kmh / 3.6
    a = (f_trac - fuerza_resistencia - f_ext_aplicada) / m
    v += a * dt
    v = max(0.0, v)

    v_kmh = v * 3.6

    diag = {
        "P": P, "I": I,
        "suma_pi": suma_pi,
        "pwm": pwm,
        "error_velocidad": err,
        "v_sensor": v_medida_kmh
    }

    return v_kmh, theta, theta_anterior, integral, err, diag

class Ventana(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulador Control Velocidad")

        self.pausa = False
        self.graph_pi = True

        main_layout = QtWidgets.QVBoxLayout()
        self.setLayout(main_layout)

        self.panel_inicio = QtWidgets.QWidget()
        inicio_layout = QtWidgets.QHBoxLayout()
        self.panel_inicio.setLayout(inicio_layout)
        inicio_layout.addWidget(QtWidgets.QLabel("Velocidad de referencia (km/h):"))
        self.spin_vref = QtWidgets.QDoubleSpinBox()
        self.spin_vref.setRange(30, 130)
        self.spin_vref.setValue(Vref_kmh)
        inicio_layout.addWidget(self.spin_vref)

        inicio_layout.addWidget(QtWidgets.QLabel("Velocidad inicial (km/h):"))
        self.spin_vinicial = QtWidgets.QDoubleSpinBox()
        self.spin_vinicial.setRange(30,130)
        self.spin_vinicial.setValue(v_kmh)
        inicio_layout.addWidget(self.spin_vinicial)

        self.btn_iniciar = QtWidgets.QPushButton("Iniciar simulación")
        self.btn_iniciar.clicked.connect(self.iniciar_simulacion)
        inicio_layout.addWidget(self.btn_iniciar)

        main_layout.addWidget(self.panel_inicio)

        pg.setConfigOptions(antialias=True)

        self.shortcut_lineas_pi = QtWidgets.QShortcut(QKeySequence("Ctrl+Alt+P"), self)
        self.shortcut_lineas_pi.activated.connect(self.alternar_PI)

        self.shortcut_controles = QtWidgets.QShortcut(QKeySequence("Ctrl+Alt+B"), self)
        self.shortcut_controles.activated.connect(self.alternar_controles)

        self.alert_label = QtWidgets.QLabel("")
        self.alert_label.setAlignment(QtCore.Qt.AlignCenter)
        self.alert_clear_timer = None
        main_layout.addWidget(self.alert_label)

        self.plot_speed = pg.PlotWidget(title="Velocidad de referencia e(t), velocidad real y(t) y velocidad medida f(t) (km/h)")
        self.line_ref = self.plot_speed.plot(pen=pg.mkPen('w',style=QtCore.Qt.DashLine), name="Vref")
        self.line_feedback = self.plot_speed.plot(pen='y', name="Vel real")
        self.line_sensor = self.plot_speed.plot(pen=pg.mkPen('g'), name="Vel medida")
        self.line_umbral_sup = self.plot_speed.plot(pen=pg.mkPen('orange',style=QtCore.Qt.DotLine, width=1.5), name="Umbral +5")
        self.line_umbral_inf = self.plot_speed.plot(pen=pg.mkPen('orange',style=QtCore.Qt.DotLine, width=1.5), name="Umbral -5")
        self.plot_speed.addLegend(offset=(5, 5))

        self.plot_error = pg.PlotWidget(title="Señal de error e(t) (km/h)")
        self.line_error_cero = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('w', style=QtCore.Qt.DashLine))
        self.plot_error.addItem(self.line_error_cero)
        self.line_error = self.plot_error.plot(pen='r')

        self.plot_controller = pg.PlotWidget(title="Salida del controlador")
        self.line_controlador_cero = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('w', style=QtCore.Qt.DashLine))
        self.plot_controller.addItem(self.line_controlador_cero)
        self.line_pwm = self.plot_controller.plot(pen=pg.mkPen(style=QtCore.Qt.DotLine), name="pwm")
        self.line_p = self.plot_controller.plot(pen='b', name='P')
        self.line_i = self.plot_controller.plot(pen='y', name='I')
        self.line_controller = self.plot_controller.plot(pen='g', name='suma_pi')
        self.plot_controller.addLegend(offset=(5, 5))

        self.line_p.setVisible(True)
        self.line_i.setVisible(True)

        self.plot_perturbacion = pg.PlotWidget(title="Perturbaciones sumadas (N)")
        self.line_perturbacion_cero = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('w', style=QtCore.Qt.DashLine))
        self.plot_perturbacion.addItem(self.line_perturbacion_cero)
        self.line_perturbacion = self.plot_perturbacion.plot(pen='m')

        main_layout.addWidget(self.plot_speed)
        main_layout.addWidget(self.plot_error)
        main_layout.addWidget(self.plot_controller)
        main_layout.addWidget(self.plot_perturbacion)

        self.controls = QtWidgets.QWidget()
        controls_layout = QtWidgets.QHBoxLayout()
        self.controls.setLayout(controls_layout)

        col_left = QtWidgets.QVBoxLayout()
        estadisticas_left_widget = QtWidgets.QWidget()
        estadisticas_left_layout = QtWidgets.QVBoxLayout()
        estadisticas_left_widget.setLayout(estadisticas_left_layout)
        estadisticas_left_layout.setContentsMargins(2, 2, 2, 2)
        estadisticas_left_layout.addWidget(QtWidgets.QLabel("<b>Valores reales</b>"))
        self.lbl_vref_small = QtWidgets.QLabel(f"Vref: {Vref_kmh:.1f} km/h")
        self.lbl_vreal_small = QtWidgets.QLabel(f"Vel real: {v_kmh:.1f} km/h")
        self.lbl_vmedida_small = QtWidgets.QLabel(f"Vel medida: {0.0:.1f} km/h")
        self.lbl_perturbacion_small = QtWidgets.QLabel(f"Perturbación: {0.0:.1f} N")
        self.lbl_error_small = QtWidgets.QLabel(f"Error: {0.0:.2f} km/h")
        estadisticas_left_layout.addWidget(self.lbl_vref_small)
        estadisticas_left_layout.addWidget(self.lbl_vreal_small)
        estadisticas_left_layout.addWidget(self.lbl_vmedida_small)
        estadisticas_left_layout.addWidget(self.lbl_perturbacion_small)
        estadisticas_left_layout.addWidget(self.lbl_error_small)
        col_left.addWidget(estadisticas_left_widget)
        #controls.addLayout(col_left)

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
        #controls.addLayout(col_right)

        btn_layout = QtWidgets.QVBoxLayout()

        self.btn_pausa = QtWidgets.QPushButton("Pausar")
        self.btn_pausa.setCheckable(False)
        self.btn_pausa.clicked.connect(self.alternar_pausa)
        btn_layout.addWidget(self.btn_pausa)

        self.btn_vref_subir = QtWidgets.QPushButton("+5 km/h")
        self.btn_vref_subir.setAutoRepeat(True)
        self.btn_vref_subir.setAutoRepeatDelay(300)
        self.btn_vref_subir.setAutoRepeatInterval(120)
        self.btn_vref_subir.clicked.connect(self.aumentar_vref)

        self.btn_vref_bajar = QtWidgets.QPushButton("-5 km/h")
        self.btn_vref_bajar.setAutoRepeat(True)
        self.btn_vref_bajar.setAutoRepeatDelay(300)
        self.btn_vref_bajar.setAutoRepeatInterval(120)
        self.btn_vref_bajar.clicked.connect(self.disminuir_vref)


        btn_layout.addWidget(self.btn_vref_subir)
        btn_layout.addWidget(self.btn_vref_bajar)
        btn_apply = QtWidgets.QPushButton("Aplicar perturbaciones")


        btn_apply.clicked.connect(self.alt_perturbacion)


        btn_layout.addWidget(btn_apply)

        self.duracion_label = QtWidgets.QLabel("Duración Ráfaga (s):")
        self.duracion_spin = QtWidgets.QSpinBox()
        self.duracion_spin.setRange(1,20)
        self.duracion_spin.setValue(3)
        btn_layout.addWidget(self.duracion_label)
        btn_layout.addWidget(self.duracion_spin)

        #controls.addLayout(btn_layout)
        controls_layout.addLayout(col_left)
        controls_layout.addLayout(col_right)
        controls_layout.addLayout(btn_layout)

        self.slider_unificado = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_unificado.setRange(-400,800)
        self.slider_unificado.setValue(0)
        self.slider_unificado.valueChanged.connect(self.cambio_perturbacion_slider)
        main_layout.addWidget(self.controls)
        self.slider_unificado_label = QtWidgets.QLabel("Perturbación (N): 0.0")
        self.slider_unificado_label.setAlignment(QtCore.Qt.AlignCenter)
        main_layout.addWidget(self.slider_unificado_label)
        main_layout.addWidget(self.slider_unificado)

        self.viento_temp = 0.0
        self.pendiente_temp = 0.0
        self.perturbacion_temp = 0.0
        self.rafaga_duracion_ms = 3000
        self._rafaga_backup = None
        self._rafaga_fuerza_backup = 0.0

        self.viento_antes_aplicar = 0.0
        self.pendiente_antes_aplicar = 0.0

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_sim)
        #self.timer.start(int(dt * 1000))

        self.rafaga_tiempo_restante = 0.0

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
            f_perturbacion = fuerza_perturbacion_actual if perturbacion_estado else 0.0
            self.lbl_perturbacion_small.setText(f"Perturbación: {f_perturbacion:.1f} N")

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
            v_medida = diag.get("v_sensor", 0.0)
            self.lbl_vmedida_small.setText(f"Vel medida: {v_medida:.1f} km/h")
        else:
            self.lbl_error_small.setText("Error: 0.00 km/h")
            self.lbl_P_small.setText("P: 0.0000 m/s")
            self.lbl_I_small.setText("I: 0.0000 m/s")
            self.lbl_suma_pi_small.setText("Suma PI: 0.0000")
            self.lbl_pwm_small.setText("PWM: 0.0000")

        self.lbl_theta_small.setText(f"Theta: {theta:.3f} (frac) / {theta*100:.1f}%")

    def cambio_perturbacion_slider(self, value):
        self.slider_unificado_label.setText(f"Perturbación (N): {float(value):.1f}")

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

    def alt_perturbacion(self):
        global perturbacion_estado, fuerza_perturbacion_actual
        valor_rafaga = float(self.slider_unificado.value())

        if perturbacion_estado:
            return

        if valor_rafaga == 0.0:
            self.mostrar_alerta("Ráfaga: el valor del slider es 0, no se aplica ráfaga.", level="info",
                                timeout_ms=3000)
            return

        self._rafaga_backup = fuerza_perturbacion_actual
        fuerza_perturbacion_actual = valor_rafaga

        self.rafaga_tiempo_restante = self.duracion_spin.value()
        perturbacion_estado = True

        duracion_s = self.duracion_spin.value()
        self.rafaga_duracion_ms = int(duracion_s * 1000)

        print(f"Ráfaga aplicada: {valor_rafaga} N por {self.rafaga_duracion_ms / 1000:.1f}s (Fuerza "
              f"{valor_rafaga:.1f} N)")
        #QtCore.QTimer.singleShot(self.rafaga_duracion_ms, self._rafaga_fin)

    def _rafaga_fin(self):
        global perturbacion_estado, fuerza_perturbacion_actual
        perturbacion_estado = False
        fuerza_perturbacion_actual = float(self._rafaga_fuerza_backup) if self._rafaga_fuerza_backup is not None else 0.0
        self._rafaga_backup = 0.0
        self.mostrar_alerta("Ráfaga terminada.", level="info", timeout_ms=1000)

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


    def _set_vref(self, nuevo_valor):
        global Vref_kmh
        Vref_kmh = float(np.clip(nuevo_valor, 30.0, 130.0))
        self._update_small_stats()
        self.mostrar_alerta(f"Vref -> {Vref_kmh:.1f} km/h", level="info", timeout_ms=1000)
        print(f"Vref -> {Vref_kmh:.1f} km/h")

    def aumentar_vref(self):
        self._set_vref(Vref_kmh + 5.0)

    def disminuir_vref(self):
        self._set_vref(Vref_kmh - 5.0)


     #simulacion
    def update_sim(self):
        global v_kmh, theta, theta_anterior, integral, prev_error, t_sim, perturbacion_estado, fuerza_perturbacion_actual

        v_kmh, theta, theta_anterior, integral, prev_error, diag \
            = aplicar_control(v_kmh, theta, theta_anterior, integral, Vref_kmh, dt)

        t_sim += dt

        fuerza_resistencia = v_kmh * b
        f_ext_aplicada = fuerza_perturbacion_actual if perturbacion_estado else 0.0

        times.append(t_sim)
        refs.append(Vref_kmh)
        vels.append(v_kmh)
        error_log.append(diag["error_velocidad"] * 3.6)
        controller_log.append(diag["suma_pi"])
        pwm_log.append(diag["pwm"])
        perturbacion_log.append(f_ext_aplicada)

        theta_real_log.append(theta * 100)
        p_log.append(diag["P"])
        i_log.append(diag["I"])
        pi_log.append(diag["suma_pi"])

        vels_medida.append(diag["v_sensor"])

        self._update_small_stats(diag=diag, fuerza_actual=f_ext_aplicada)

        self.line_ref.setData(times, refs)
        self.line_feedback.setData(times, vels)
        self.line_sensor.setData(times, vels_medida)

        if len(times) > 0:
            umbral_sup = [Vref_kmh + 5.0] * len(times)
            umbral_inf = [Vref_kmh - 5.0] * len(times)

            self.line_umbral_sup.setData(times, umbral_sup)
            self.line_umbral_inf.setData(times, umbral_inf)

        self.line_error.setData(times, error_log)

        self.line_controller.setData(times, controller_log)
        self.line_p.setData(times, p_log)
        self.line_i.setData(times, i_log)
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
            cmin = min(min(controller_log), min(pwm_log), min(p_log), min(i_log)) if self.graph_pi else min(min(controller_log), min(pwm_log))
            cmax = max(max(controller_log), max(pwm_log), max(p_log), max(i_log)) if self.graph_pi else max(max(controller_log), max(pwm_log))
            if cmin == cmax:
                self.plot_controller.setYRange(cmin - 0.1, cmax + 0.1, padding=0)
            else:
                self.plot_controller.setYRange(cmin - 0.2 * abs(cmin), cmax + 0.2 * abs(cmax), padding=0)

        if len(perturbacion_log) > 0:
            pmin = min(perturbacion_log)
            pmax = max(perturbacion_log)
            if pmax == pmin:
                self.plot_perturbacion.setYRange(pmin - 10, pmax + 10)
            else:
                self.plot_perturbacion.setYRange(pmin - 0.1 * abs(pmin), pmax + 0.1 * abs(pmax))

        if perturbacion_estado:
            self.rafaga_tiempo_restante -= dt
            if self.rafaga_tiempo_restante <= 0:
                perturbacion_estado = False
                fuerza_perturbacion_actual = 0.0

    def alternar_pausa(self):
        if self.pausa:
            self.timer.start(int(dt * 1000))
            self.btn_pausa.setText("Pausar")
            self.mostrar_alerta("Simulación reanudada.", level="info", timeout_ms = 1200)
            self.pausa = False
        else:
            self.timer.stop()
            self.btn_pausa.setText("Reanudar")
            self.mostrar_alerta("Simulación pausada.", level="info", timeout_ms = 1200)
            self.pausa = True

    def alternar_PI(self):
        self.graph_pi = not self.graph_pi
        self.line_p.setVisible(self.graph_pi)
        self.line_i.setVisible(self.graph_pi)

    def alternar_controles(self):
        visible = not self.controls.isVisible()
        self.controls.setVisible(visible)
        self.slider_unificado.setVisible(visible)
        self.slider_unificado_label.setVisible(visible)

    def iniciar_simulacion(self):
        global Vref_kmh, v_kmh
        Vref_kmh = self.spin_vref.value()
        v_kmh = self.spin_vinicial.value()
        self.panel_inicio.setVisible(False)
        self.timer.start(int(dt * 1000))

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()

    def keyPressEvent(self, event):
        global Vref_kmh
        if event.key() == QtCore.Qt.Key_Plus or event.key() == QtCore.Qt.Key_Equal:
            #Vref_kmh = min(Vref_kmh + 5, 130.0)
            #print(f"Vref -> {Vref_kmh:.1f} km/h")
            self.aumentar_vref()
        elif event.key() == QtCore.Qt.Key_Minus:
            #Vref_kmh = max(Vref_kmh - 5, 30.0)
            #print(f"Vref -> {Vref_kmh:.1f} km/h")
            self.disminuir_vref()
        '''elif event.key() == QtCore.Qt.Key_P:
            self.alternar_PI()'''

app = QtWidgets.QApplication(sys.argv)
ventana = Ventana()
sys.exit(app.exec_())

