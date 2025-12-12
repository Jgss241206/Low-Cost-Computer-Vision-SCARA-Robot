# **Low-Cost Arduino CNC Arm with Computer Vision**

A 2-DOF SCARA robotic arm controlled in real-time using Computer Vision (Hand Tracking).  
Built with affordable materials to demonstrate Inverse Kinematics and Human-Machine Interface (HMI) concepts.

### **English Documentation**

## **Features**

* **Computer Vision Control:** Uses MediaPipe to track the index finger via webcam.  
* **Inverse Kinematics:** Real-time calculation of servo angles based on (X, Y) coordinates.  
* **Wireless/USB Communication:** Python script sends serial data to Arduino via USB.  
* **Cost-Effective Design:** Built using SG90 servos and accessible materials (wood/PLA).

## **Hardware Requirements**

* 1x Arduino Uno (or Nano/Mega)  
* 2x Micro Servos (SG90 or MG90S)  
* 1x Webcam (Built-in or USB)  
* Jumper wires & Breadboard  
* External 5V Power Supply (Optional but recommended for stability)

### **Wiring Diagram**

| Component | Arduino Pin | Note |
| :---- | :---- | :---- |
| **Servo 1** (Shoulder) | Pin 9 | You can use whichever PWM pin you want |
| **Servo 2** (Elbow) | Pin 10 | You can use whichever PWM pin you want |
| **Servo Power** | 5V | External power recommended |
| **Servo GND** | GND | Common ground is vital |

## **Setup & Installation**

### **1\. Arduino Firmware**

1. Open the CNC\_Serial\_Receiver.ino file in Arduino IDE.  
2. Update the L1 and L2 constants with your arm measurements (in cm).  
3. Upload to your board.  
4. **Important:** Close the Arduino IDE to free up the COM port.

### **2\. Python Controller**

**Prerequisites:**

pip install opencv-python mediapipe pyserial

**Running the controller:**

1. Connect the Arduino via USB.  
2. Check your COM port (e.g., COM3 on Windows or /dev/ttyUSB0 on Linux).  
3. Update the PUERTO\_ARDUINO variable in CNC\_Vision\_Control.py.  
4. Run the script:  
   python CNC\_Vision\_Control.py

## **The Math (Inverse Kinematics)**

The robot uses the **Law of Cosines** to calculate the required angles ($\\theta\_1, \\theta\_2$) to reach a target coordinate $(x, y)$.

$$ c^2 \= a^2 \+ b^2 \- 2ab \\cdot \\cos(\\gamma) $$

Where:

* $a, b$: Arm lengths ($L\_1, L\_2$)  
* $c$: Distance to target ($\\sqrt{x^2 \+ y^2}$)

## **Future Roadmap**

* \[ \] Add Z-axis (Servo for pen lifting).  
* \[ \] Implement "Air Drawing" (Draw gestures in the air and replicate on paper).  
* \[ \] Voice Control integration.

### **Documentación en Español**

## **Resumen**

Este proyecto es una implementación de un brazo robótico tipo SCARA de bajo costo, controlado por gestos de la mano en tiempo real. Demuestra que se pueden aplicar conceptos avanzados de robótica (Cinemática Inversa, Visión Artificial) con hardware accesible.

## **Cómo funciona**

1. **Cámara:** Detecta la punta de tu dedo índice usando Inteligencia Artificial (MediaPipe).  
2. **Mapeo:** Traduce la posición de tu dedo en la pantalla (pixeles) a coordenadas físicas del robot (centímetros).  
3. **Matemáticas:** El Arduino recibe (X, Y) y calcula qué tanto deben girar los motores para llegar a ese punto exacto usando trigonometría.

## **Solución de Problemas (Troubleshooting)**

* **El robot no se mueve:** Revisa que el LED RX del Arduino parpadee. Si no, cierra el Monitor Serie del Arduino IDE.  
* **Los servos vibran mucho:** Probablemente les falta corriente. Usa una fuente externa de 5V (o un Power Bank USB).  
* **Movimiento errático:** Verifica que las longitudes L1 y L2 en el código sean idénticas a las medidas reales de tus palitos.

## **Author / Autor**

Jorge Gael Santiago Simón  
Engineering Student at UNAM | Mechatronics Enthusiast  
LinkedIn Profile  [https://www.linkedin.com/in/jorge-gael-santiago-sim%C3%B3n/]
*Made with passion in Mexico City.*
