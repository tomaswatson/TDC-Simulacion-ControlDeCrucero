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

Para cambiar la velocidad de referencia se utilizan las teclas:
- '+' para incrementarla.
- '-' para disminuirla.

### Perturbaciones

El sistema permite aplicar perturbaciones externas durante la ejecución. Estas perturbaciones se configuran mediante los controles en la interfaz:
- Sliders de viento y pendiente:
  
  Permiten seleccionar la magnitud de cada perturbación.
- Botón "Aplicar perturbación":
  
  Activa la perturbación en función al valor elegido en los sliders.
- Botón "Ráfaga":
  
  Genera una ráfaga de viento de 3 segundos con la magnitud seleccionada.
- Botón "Reset perturbaciones":
  
  Establece todas las perturbaciones a cero.
- Casilla "Actualizar en vivo":
  
  Si está habilitada, los sliders tienen efecto inmediato en el sistema. 
