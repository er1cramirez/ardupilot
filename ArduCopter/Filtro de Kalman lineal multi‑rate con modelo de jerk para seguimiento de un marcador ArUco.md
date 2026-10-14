# Filtro de Kalman lineal multi‑rate con modelo de jerk para seguimiento de un marcador ArUco

## 1. Planteamiento del problema

Se desea estimar posición, velocidad y aceleración lineales de un marcador ArUco en 3D a partir de mediciones de visión (pose relativa) con frecuencia variable (~30–40 Hz, con jitter), mientras que la predicción del filtro corre a una frecuencia fija más alta (por ejemplo, 100 Hz) sincronizada con el lazo de control del sistema (p. ej., ArduPilot). Las mediciones de visión incluyen timestamp y pueden llegar de forma asíncrona respecto al ciclo de predicción del filtro.

Este escenario encaja en la clase de problemas de **seguimiento de blancos con sensores multi‑rate y muestreo irregular**, donde la dinámica del objetivo se modela con un modelo de movimiento (en este caso, con **jerk**) y las mediciones llegan a distinta frecuencia, incluso con pequeñas variaciones de periodo y latencia.[^1][^2][^3]

El objetivo es diseñar un **Kalman Filter lineal discreto** (no EKF) con:

- Modelo de estado lineal extendido que incluya jerk para mejorar la estimación de aceleración.[^4][^5]
- Predicción a frecuencia fija (task de scheduler) utilizando el \(\Delta t\) real entre iteraciones.
- Actualizaciones de medida de visión asíncronas basadas en timestamps, sin necesidad inicial de correcciones retardadas (retrodiction/OOSM), dado que las latencias y jitter son moderados.[^6][^2][^7]


## 2. Revisión teórica relevante

### 2.1 KF vs EKF en este contexto

El Kalman Filter lineal se aplica cuando tanto la dinámica como el modelo de medida son lineales en el estado, con ruido gaussiano. El Extended Kalman Filter se reserva para el caso en que la dinámica \(f(\cdot)\) o la medida \(h(\cdot)\) sean no lineales y se requiera linealización mediante Jacobianos.[^8][^9][^10][^4]

En el problema propuesto:

- La dinámica de movimiento con jerk se puede escribir como un sistema lineal en el estado (matriz \(F(\Delta t)\)).[^5]
- Si la salida de ArUco se transforma a posición cartesiana en un frame fijo (mundo o dron), el modelo de medida es lineal en el estado cuando solo se observa la posición (\(z = H x + v\)).[^11][^12][^13]

Por ello, un **KF lineal con modelo de jerk** es suficiente y más simple que un EKF, evitando la complejidad de Jacobianos y errores de linealización innecesarios.[^14][^15]


### 2.2 Modelos de movimiento con aceleración y jerk

En seguimiento de blancos se utilizan ampliamente modelos cinemáticos discretizados: velocidad constante (CV), aceleración constante (CA) y jerk (derivada de la aceleración). Estos modelos se usan incluso cuando la dinámica exacta del objetivo no se conoce, siempre que el movimiento sea relativamente suave y acotado.[^16][^17][^5]

Trabajos que comparan CV, CA y jerk muestran que:

- El modelo CA mejora la estimación de posición/velocidad frente a CV durante maniobras, pero la estimación de aceleración puede mostrar sesgos significativos cuando las maniobras son dinámicas.[^5]
- Los modelos con jerk (estado \([p,v,a,j]\)) proporcionan estimaciones de aceleración más precisas, reduciendo sesgo y retraso, especialmente en blancos con maniobras moderadas o altas.[^18][^19][^5]

Dado que en el sistema considerado es importante obtener una **buena estimación de la aceleración** para uso en control (feed‑forward, compensaciones, etc.), el uso de un modelo con jerk resulta adecuado.


### 2.3 Multi‑rate, muestreo irregular y asincronía

La literatura de Kalman en sistemas multi‑rate y con muestreo irregular indica que un KF estándar puede manejar variaciones en el periodo de muestreo si se utiliza el \(\Delta t\) real para construir las matrices de transición \(F(\Delta t)\) y de ruido de proceso \(Q(\Delta t)\). No es imprescindible que el periodo sea exactamente constante.[^2][^3]

En sistemas donde diferentes sensores operan a frecuencias distintas, la práctica habitual consiste en:

- Ejecutar la **predicción** a la frecuencia del lazo principal (p. ej., lazo de control o sensor más rápido).
- Ejecutar la **actualización** únicamente cuando hay mediciones nuevas disponibles para cada sensor, usando sus timestamps para decidir en qué instante del timeline aplicarlas.[^7][^1]

Las técnicas de mediciones fuera de secuencia (OOSM) y retrodiction se vuelven relevantes cuando las mediciones llegan con retrasos significativos respecto al estado actual y se requiere máxima precisión, pero incrementan la complejidad de implementación. En el escenario con visión 30–40 Hz y latencias moderadas, suele ser suficiente aplicar las mediciones en el instante de recepción y absorber errores de retardo en las covarianzas \(Q,R\).[^20][^21][^22][^6]


## 3. Modelo de estado con jerk

### 3.1 Definición del estado 1D y dinámica continua

Para un eje (por ejemplo, x), se define el estado como:

\[
 x = \begin{bmatrix} p \\ v \\ a \\ j \end{bmatrix}
\]

con dinámica continua:

\[
 \dot{p} = v, \quad \dot{v} = a, \quad \dot{a} = j, \quad \dot{j} = 0
\]

En la práctica, \(j\) no es estrictamente constante; se modela como casi constante más ruido (jerk random walk o proceso de primer orden) para capturar maniobras. Esto es coherente con modelos usados en filtros \(\alpha\text{-}\beta\text{-}\gamma\text{-}\eta\) y en trabajos de seguimiento de blancos de alta dinámica.[^19][^23][^5]


### 3.2 Discretización exacta (modelo jerk constante en el intervalo)

Asumiendo jerk constante en un intervalo de tiempo \(\Delta t\), la solución discreta de las ecuaciones anteriores da:

\[
\begin{aligned}
 p_{k+1} &= p_k + v_k \Delta t + \tfrac{1}{2} a_k \Delta t^2 + \tfrac{1}{6} j_k \Delta t^3, \\
 v_{k+1} &= v_k + a_k \Delta t + \tfrac{1}{2} j_k \Delta t^2, \\
 a_{k+1} &= a_k + j_k \Delta t, \\
 j_{k+1} &= j_k.
\end{aligned}
\]

Esto puede escribirse como un sistema lineal discreto:

\[
 x_{k+1} = F_{1D}(\Delta t)\,x_k + w_k,
\]

donde la matriz de transición 1D es:[^18][^5]

\[
F_{1D}(\Delta t)=
\begin{bmatrix}
1 & \Delta t & \tfrac{1}{2}\Delta t^2 & \tfrac{1}{6}\Delta t^3 \\
0 & 1        & \Delta t               & \tfrac{1}{2}\Delta t^2 \\
0 & 0        & 1                      & \Delta t \\
0 & 0        & 0                      & 1
\end{bmatrix}.
\]

Para 3D se apilan tres copias de este estado (x,y,z):

\[
 x = \begin{bmatrix} p_x \\ v_x \\ a_x \\ j_x \\ p_y \\ v_y \\ a_y \\ j_y \\ p_z \\ v_z \\ a_z \\ j_z \end{bmatrix}
\]

con matriz de transición

\[
 F(\Delta t) = \operatorname{blockdiag}\big(F_{1D}(\Delta t), F_{1D}(\Delta t), F_{1D}(\Delta t)\big).
\]

Este modelo es lineal respecto al estado y, por tanto, compatible con un KF lineal clásico.


### 3.3 Ruido de proceso \(Q(\Delta t)\)

Hay varias formas de construir \(Q(\Delta t)\) para este modelo. En la literatura, se suele:

- Tratar el jerk como random walk excitado por ruido blanco, con varianza \(\sigma_j^2\), y derivar \(Q(\Delta t)\) a partir de las integrales de ese ruido a través de la dinámica, lo que produce expresiones analíticas en función de potencias de \(\Delta t\).[^19][^5]
- O aproximar con una matriz diagonal o casi diagonal, donde los elementos crecen con potencias de \(\Delta t\) (mayor contribución al error en posición que en jerk), escalados por parámetros \(\sigma_p^2, \sigma_v^2, \sigma_a^2, \sigma_j^2\) ajustados experimentalmente.[^24][^16]

Para una primera implementación práctica y dado que se busca simplicidad, se puede adoptar una aproximación diagonal:

\[
 Q_{1D}(\Delta t) = \operatorname{diag}\left(q_p(\Delta t), q_v(\Delta t), q_a(\Delta t), q_j(\Delta t)\right),
\]

con, por ejemplo:

\[
 q_p(\Delta t) = k_p \Delta t^5, \quad q_v(\Delta t) = k_v \Delta t^4, \quad q_a(\Delta t) = k_a \Delta t^3, \quad q_j(\Delta t) = k_j \Delta t^2,
\]

inspirado en la dependencia por potencias de \(\Delta t\) que aparece en las derivaciones teóricas de modelos con jerk. Los coeficientes \(k_p, k_v, k_a, k_j\) se ajustan a partir de las maniobras máximas esperadas (velocidad tangencial, radio, jerk máximo admisible).[^16][^5]

En 3D se utiliza

\[
 Q(\Delta t) = \operatorname{blockdiag}\big(Q_{1D}(\Delta t), Q_{1D}(\Delta t), Q_{1D}(\Delta t)\big).
\]

Una vez validada la implementación, se puede refinar \(Q(\Delta t)\) usando las expresiones cerradas de la literatura para jerk excitado por ruido blanco si se requiere mayor exactitud.[^5]


## 4. Modelo de medida (posición de ArUco)

Suponiendo que el pipeline de visión proporciona la posición del marcador en un frame fijo \(\tilde{p} = (\tilde{p}_x, \tilde{p}_y, \tilde{p}_z)\) mediante PnP y transformaciones conocidas, el modelo de medida es:

\[
 z_k = \begin{bmatrix} \tilde{p}_x \\ \tilde{p}_y \\ \tilde{p}_z \end{bmatrix} = H x_k + v_k,
\]

con \(v_k \sim \mathcal{N}(0,R)\) y \(H\) dado por:

\[
 H =
 \begin{bmatrix}
 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\
 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0\\
 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0
 \end{bmatrix}.
\]

La matriz \(R\) recoge la covarianza de la posición estimada por ArUco; se puede tomar inicialmente diagonal con varianzas basadas en el error medio de reproyección y refinarla haciéndola dependiente de la geometría de observación (distancia, número de marcadores, ángulos), como se propone en trabajos de posicionamiento basado en ArUco.[^12][^13][^11]

Dado que la relación entre el estado y la medición es lineal, el uso de un KF lineal sigue siendo apropiado.


## 5. Estructura multi‑rate: predicción a frecuencia fija y updates asíncronos

### 5.1 Organización general

La organización propuesta sigue el patrón estándar en sistemas multi‑rate:

- Un hilo o task de alta frecuencia (p. ej., 100 Hz) ejecuta la función `compute()`, que realiza el paso de predicción y consume las mediciones disponibles cuyo timestamp sea anterior o igual al tiempo actual.[^1][^7]
- Un hilo de visión (o proceso externo) corre el pipeline ArUco a ~30–40 Hz, calcula la posición del marcador y encola mediciones \((z, R, t_m)\) en una cola thread‑safe.

Este esquema está alineado con enfoques multi‑rate descritos en la literatura para filtros de Kalman con sensores a distintas frecuencias y muestreo no uniforme.[^3][^2][^1]


### 5.2 Estructuras de datos

Se asume una estructura de estado del filtro:

```text
struct EKFState {
    VectorX x;       // 12x1: [px vx ax jx py vy ay jy pz vz az jz]^T
    MatrixX P;       // 12x12, covarianza
    double last_time; // [s], instante del último compute()
};
```

y una cola de mediciones de visión:

```text
struct VisionMeas {
    Vector3 z;   // posición medida (px, py, pz)
    Matrix3 R;   // covarianza de la medida
    double t;    // timestamp [s] asociado a la medición
};

lockfree_queue<VisionMeas> vision_queue; // producida por el hilo de visión
```


### 5.3 Función `compute(now, state)`

La función `compute` se llama a frecuencia fija por el scheduler (por ejemplo, desde `Mode::run()` en ArduPilot), y realiza:

1) Cálculo de \(\Delta t\) real y construcción de \(F(\Delta t)\) y \(Q(\Delta t)\).
2) Paso de predicción.
3) Consumo de mediciones con timestamp \(t_m \le \text{now}\) y llamada a `measurement_update` por cada una.
4) Publicación del estado para el controlador.

Pseudocódigo:

```text
void compute(double now, EKFState &s) {
    double dt = now - s.last_time;
    if (dt <= 0) {
        dt = small_positive; // evitar dt=0
    }

    // 1. Construir F(dt) y Q(dt) para modelo de jerk
    MatrixX F = make_F_jerk(dt);   // 12x12, blockdiag(F1D, F1D, F1D)
    MatrixX Q = make_Q_jerk(dt);   // 12x12, dependiente de dt y parámetros de ruido

    // 2. Predicción
    s.x = F * s.x;
    s.P = F * s.P * F.transpose() + Q;

    // 3. Consumir mediciones de visión con t <= now
    VisionMeas m;
    while (vision_queue.peek(m) && m.t <= now) {
        vision_queue.pop(m);
        measurement_update(s, m.z, m.R);
    }

    s.last_time = now;

    // 4. Publicar estado (p, v, a) para el controlador
    publish_state_to_controller(s.x);
}
```

Este esquema es consistente con las recomendaciones para KF en sistemas con muestreo no uniforme, donde se usa \(\Delta t\) real en la dinámica y se realizan actualizaciones cuando hay mediciones disponibles.[^2][^3][^1]


### 5.4 Función `measurement_update(state, z, R)`

La actualización de medida es la del KF lineal estándar:

1) Innovación: \(y = z - H x\).
2) Covarianza de innovación: \(S = H P H^T + R\).
3) Ganancia de Kalman: \(K = P H^T S^{-1}\).
4) Actualización de estado y covarianza:

\[
 x \leftarrow x + K y, \quad P \leftarrow (I - K H) P.
\]

Pseudocódigo:

```text
void measurement_update(EKFState &s, const Vector3 &z, const Matrix3 &R) {
    // H es constante, 3x12, precalculada
    static const Matrix<double,3,12> H = make_H_position_only();

    // 1. Innovación
    Vector3 z_pred = H * s.x;  // predicción de posición
    Vector3 y      = z - z_pred;

    // 2. Covarianza de innovación
    Matrix3 S = H * s.P * H.transpose() + R;

    // 3. Ganancia de Kalman
    MatrixX K = s.P * H.transpose() * S.inverse(); // 12x3

    // 4. Actualización
    s.x = s.x + K * y;
    MatrixX I = MatrixX::Identity(12,12);
    s.P = (I - K * H) * s.P;
}
```

Esta función está completamente desacoplada de la frecuencia de ejecución; se ejecuta cada vez que `compute()` decide aplicar una medición, lo que permite manejar mediciones de visión a frecuencia variable y asíncrona.


## 6. Construcción explícita de \(F(\Delta t)\) y bosquejo de \(Q(\Delta t)\)

### 6.1 Función `make_F_jerk(dt)`

Para un eje (1D), se define:

```text
Matrix4 F1D(double dt) {
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;

    Matrix4 F;
    F << 1.0, dt,   0.5*dt2, (1.0/6.0)*dt3,
         0.0, 1.0,  dt,      0.5*dt2,
         0.0, 0.0,  1.0,     dt,
         0.0, 0.0,  0.0,     1.0;
    return F;
}
```

Para 3D, `make_F_jerk(dt)` arma la block‑diagonal:

```text
MatrixX make_F_jerk(double dt) {
    Matrix4 Fx = F1D(dt);

    MatrixX F(12,12);
    F.setZero();

    F.block<4,4>(0,0)   = Fx; // x
    F.block<4,4>(4,4)   = Fx; // y
    F.block<4,4>(8,8)   = Fx; // z

    return F;
}
```

Esta forma de \(F(\Delta t)\) coincide con las expresiones derivadas para modelos de jerk constante en la literatura de filtros de seguimiento.[^18][^5]


### 6.2 Bosquejo de `make_Q_jerk(dt)`

Una opción sencilla, inspirada en las dependencias temporales de los modelos con jerk, es:

```text
Matrix4 Q1D(double dt) {
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt2 * dt2;
    double dt5 = dt3 * dt2;

    // Coeficientes de ruido (ajustables)
    double kp = sigma_p2; // escala para posición
    double kv = sigma_v2; // escala para velocidad
    double ka = sigma_a2; // escala para aceleración
    double kj = sigma_j2; // escala para jerk

    Matrix4 Q;
    Q.setZero();
    Q(0,0) = kp * dt5;  // contribución dominante en p
    Q(1,1) = kv * dt4;  // en v
    Q(2,2) = ka * dt3;  // en a
    Q(3,3) = kj * dt2;  // en j

    return Q;
}

MatrixX make_Q_jerk(double dt) {
    Matrix4 Qx = Q1D(dt);

    MatrixX Q(12,12);
    Q.setZero();

    Q.block<4,4>(0,0)   = Qx;
    Q.block<4,4>(4,4)   = Qx;
    Q.block<4,4>(8,8)   = Qx;

    return Q;
}
```

Aunque esta \(Q\) es diagonal y simplificada, sigue la idea de que la incertidumbre en posición crece con una potencia mayor de \(\Delta t\) que la de velocidad, aceleración y jerk, de forma análoga a las derivaciones más rigurosas. Los parámetros \(\sigma_p^2, \sigma_v^2, \sigma_a^2, \sigma_j^2\) se ajustan en función de la dinámica esperada (por ejemplo, velocidades tangenciales hasta ~4–5 m/s y radios de 1–3 m) y se afinan empíricamente observando los residuos del filtro.[^16][^5]

Si más adelante se requiere, se puede sustituir `Q1D` por las expresiones exactas de \(Q(\Delta t)\) derivadas para un modelo de jerk excitado por ruido blanco, como se expone en estudios específicos de modelos de jerk.[^5]


## 7. Conclusiones

- Para el seguimiento de un marcador ArUco con necesidad de buena estimación de aceleración, un **modelo de jerk** en un KF lineal es adecuado y está respaldado por la literatura de seguimiento de blancos y filtros de orden superior.[^19][^18][^5]
- Dado que las mediciones de visión llegan a ~30–40 Hz con jitter moderado y la predicción del filtro se ejecuta a frecuencia fija más alta, un enfoque **multi‑rate** con \(\Delta t\) variable y actualizaciones asíncronas es coherente con trabajos sobre Kalman en sistemas no uniformemente muestreados.[^3][^1][^2]
- El modelo resultante es lineal tanto en la dinámica como en la medida (posición cartesiana), por lo que un **KF lineal** es suficiente; no se requiere EKF mientras no se introduzcan medidas o estados no lineales (por ejemplo, orientación con quaterniones o medidas angulares directas).[^10][^25][^8]
- La separación clara entre `compute()` (predicción a frecuencia fija y consumo de cola) y `measurement_update()` (update de medida) facilita la integración en frameworks como ArduPilot, donde el scheduler ofrece tasks a frecuencia fija y los hilos de visión pueden operar de forma independiente.

Este marco teórico y de implementación proporciona una base sólida para desarrollar, integrar y depurar el filtro, así como para futuras extensiones (modelos Singer, retrodiction, integración con IMU, etc.).

---

## References

1. [Asynchronous Kalman filtering for dynamic response reconstruction ...](https://www.sciencedirect.com/science/article/abs/pii/S0888327024002930) - This study proposes a state-of-the-art asynchronous Kalman filtering (ASKF) technique for reconstruc...

2. [Target Tracking of a Linear Time Invariant System under Irregular ...](https://journals.sagepub.com/doi/10.5772/54471) - Using the common Kalman filter, the developed method is used to track a target for the simulated tra...

3. [[PDF] Kalman Filters in Non-uniformly Sampled Multirate Systems: for FDI ...](https://sites.ualberta.ca/~slshah/public_share/weihua/mr_kalman_fdi_rev.pdf) - First, this paper attempts to develop the Kalman filter for a NUSM system, including algorithms for ...

4. [Kalman Filter for Motion Prediction - Emergent Mind](https://www.emergentmind.com/topics/kalman-filter-for-motion-prediction) - Kalman Filter for Motion Prediction is a recursive state estimator that uses linear Gaussian models ...

5. [[PDF] Modeling and Analysis af Velocity, Acceleration and Jerk Models For ...](https://acadpubl.eu/jsi/2018-118-18/articles/18d/92.pdf) - Kalman Filter (KF) is extensively used in target tracking applications due to its simplicity, optima...

6. [Incorporating delayed and infrequent measurements in Extended ...](https://www.sciencedirect.com/science/article/abs/pii/S095915241000199X) - Kalman filtering approach to multi-rate information fusion in the presence of irregular sampling rat...

7. [Kalman filter with different sampling rate - Stack Overflow](https://stackoverflow.com/questions/59566384/kalman-filter-with-different-sampling-rate) - My question is now, how do you (optimally) use a Kalman filter when the sampling rate from the diffe...

8. [Kalman Filter Explained Through Examples](https://kalmanfilter.net) - The Kalman Filter is an algorithm for estimating and predicting the state of a system in the presenc...

9. [[PDF] 16.4 Extended Kalman Filter - Carnegie Mellon University](https://www.cs.cmu.edu/~16385/s17/Slides/16.4_Extended_Kalman_Filter.pdf) - Does not assume linear Gaussian models. • Assumes Gaussian noise. • Uses local linear approximations...

10. [Extended Kalman filter - Wikipedia](https://en.wikipedia.org/wiki/Extended_Kalman_filter) - In estimation theory, the extended Kalman filter (EKF) is the nonlinear version of the Kalman filter...

11. [Investigation of ArUco Marker Placement for Planar Indoor ... - arXiv](https://arxiv.org/html/2509.17345v1) - The linear Kalman filter relies on the measurements to update the AMR velocity and performance will ...

12. [[PDF] Relative Vessel Motion Tracking using Sensor Fusion, Aruco ...](https://pdfs.semanticscholar.org/d258/cc77dfcbc3e0234fcc628b48e1bf20e91907.pdf) - This paper presents a novel approach for estimating the relative motion between two moving offshore ...

13. [A Mobile Robot Indoor Positioning System Based on ArUco Array ...](https://www.semanticscholar.org/paper/A-Mobile-Robot-Indoor-Positioning-System-Based-on-Jiang-Ma/2780b7659f0371ebe68f5bd6a88f327461fee430) - A positioning method using an ArUco marker array as a manual marker and a camera as a global absolut...

14. [[PDF] Using Kalman filters for pose estimation of mobile devices ... - IS MUNI](https://is.muni.cz/th/ds9vh/FullTextDigital.pdf) - The focus is on marker-based visual-inertial pose tracking systems that use Kalman filtering for sen...

15. [Kalman Filter vs EKF : r/robotics - Reddit](https://www.reddit.com/r/robotics/comments/25m1zb/kalman_filter_vs_ekf/) - The only difference between EKF and KF is that the EKF is for a non-linear model. It uses the linear...

16. [[PDF] ADAPTIVE KALMAN FILTER APPROACHES IN TARGET TRACKING](https://trepo.tuni.fi/bitstream/10024/159132/5/RautiainenSimo.pdf) - Standard linear Kalman filters are unsuitable for non-linear measurement models such as the angle me...

17. [[PDF] Models in Target Tracking - Automatic Control](https://rt.isy.liu.se/student/graduate/targettracking/file/le2_handout.pdf) - • (Nearly) constant acceleration (CA) model xt =. xt vx t ax t ... Singer Acceleration Model. Consid...

18. [[PDF] Target Tracking a Non-Linear Target Path Using Kalman Predictive ...](http://mocha-java.uccs.edu/dossier/research/2006CASA-.pdf) - This model is using the change of acceleration, or jerk, as the component driving the motion. The tw...

19. [A Study of Optimization of Alpha-Beta-Gamma-Eta Filter for Tracking ...](https://www.transnav.eu/Article_A_Study_of_Optimization_of_Alpha-Beta-Gamma-Eta_Jeong,41,697.html) - The Alpha-Beta-Gamma filter is extended to include the fourth state that is, constant jerk to correc...

20. [Kalman filter with Out-of-Sequence measurements](https://stonesoup.readthedocs.io/en/v1.4/auto_examples/oosm/KalmanFilterOOSMExample.html) - In this simple example we have presented how it is possible to perform the tracking with the presenc...

21. [Handle Out-of-Sequence Measurements with Filter Retrodiction](https://www.mathworks.com/help/fusion/ug/handle-out-of-sequence-measurements-with-filter-retrodiction.html) - This example shows how to handle out-of-sequence measurements using the retrodiction technique at th...

22. [[PDF] Incorporation of Time Delayed Measurements in a Discrete-time ...](https://backend.orbit.dtu.dk/ws/files/4363506/Larsen.pdf) - This paper considers the problem of designing discrete- time Kalman filters to systems where some re...

23. [The alpha-beta-Gamma tracking filter with a noisy jerk as the ...](https://i-rep.emu.edu.tr/items/548529eb-9992-4f09-abe6-65a928a3b157) - An analysis of a discrete constant gain alpha-beta-Gamma tracking filter is presented in frequency d...

24. [[PDF] Extended Kalman Filters at high update rates - KVMO](https://www.kvmo.nl/images/pdf/2016/ScriptieLouisElling.pdf) - The angular velocity is added to the state vector and the transition matrix is changed to cope with ...

25. [Pose Estimation of Aruco Marker with Kalman Filter - GitHub](https://github.com/AngeloDamante/pose-estimation-kalman) - A simple Kalman Filter is based on two steps, update and prediction . kf. In this project aims to im...

