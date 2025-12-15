# Sistema de control de crucero - Simulación
## Requisitos

- python
- numpy
- PyQt5
- pyqtgraph

## Instalación de requisitos

### Primer paso: Instalar Python

Descargar Python desde https://www.python.org/ y ejecutar el archivo de instalación.
Durante la instalación habilitar el casillero "Add Python to PATH".

### Segundo paso: Clonar el repositorio

Desde CMD, Powershell o Git Bash:

```bash
git clone https://github.com/tomaswatson/TDC-Simulacion-ControlDeCrucero.git
```

Ingresar al proyecto:

```bash
cd TDC-Simulacion-ControlDeCrucero
```

### Tercer paso: Crear entorno virtual (opcional pero recomendado)

Primero para aislar las dependencias, se crea un entorno virtual:

```bash
python -m venv venv
```

Segundo para activar el entorno virtual:

```bash
venv\Scripts\activate.bat
```

### Cuarto paso: Instalar dependencias

```bash
pip install PyQt5 pyqtgraph numpy
```

## Ejecucion del código

Dentro del directorio del proyecto ejecutar:

```bash
python simulacion.py
```

## Controles

Para cambiar la velocidad de referencia se pueden usar las teclas:
- '+' para incrementarla.
- '-' para disminuirla.

O se pueden utilizar los botones en pantalla:
- Botón "+5 km/h" para incrementarla.
- Botón "-5 km/h" para disminuirla.

También se cuenta con un botón "Pausar/Reanudar" que permite detener y retomar la simulación.

### Perturbaciones

El sistema permite aplicar perturbaciones externas durante la ejecución. Estas perturbaciones se configuran mediante los controles en la interfaz:
- Slider de perturbación:
  
  Permite seleccionar la magnitud de la perturbación.
- Selector numérico:

  Permite determinar la duración de la perturbación.
- Botón "Aplicar perturbación":
  
  Activa la perturbación en función al valor elegido en el slider y el selector numérico.

#### Aclaración sobre las perturbaciones
- **Perturbación positiva:** actúa en contra del movimiento del vehículo, aumentando la resistencia.
- **Perturbación negativa:** reduce la fuerza de resistencia (el vehículo tiende a acelerar).

### Ajustar coeficientes Kp y Ki

Se permite la modificación de los coeficientes Kp y Ki mediante los sliders correspondientes. Estos cambios tienen efecto inmediato en la simulación.

### Controles especiales

Además de los controles visibles en la interfaz, existen atajos que permiten mostrar u ocultar determinados elementos visuales sin afectar el funcionamiento interno de la simulación.

- Ctrl + Alt + P
Muestra u oculta los componentes P e I en la gráfica de salida del controlador.
- Ctrl + Alt + B
Muestra u oculta los controles inferiores de la interfaz.

## Gráficos

### Primer gráfico

- Línea blanca punteada: Velocidad de referencia r(t)
- Línea amarilla: Velocidad real y(t)
- Línea verde: Velocidad medida f(t)
- Líneas naranja punteadas: Umbral de falla

### Segundo gráfico

- Línea roja: Señal de error e(t)

### Tercer gráfico

- Línea verde: Salida del controlador
- Línea azul: Componente proporcional
- Línea amarilla: Componente integral

### Cuarto gráfico

- Línea violeta: Valor de la perturbación
