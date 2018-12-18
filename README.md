# Quadcopter



## Система уравнений, описывающая движение квадрокоптера.
Первые три уравнения -- законы поступательного движения (вторые законы Ньютона).
Последние три уравнения -- законы вращательного движения.

![System of equations of movement](res/12.png)

где

![Equations (5)-(8)](res/5-8.png)



## Integrated PD controller

Current coordinates: x, y, z <br>
Desired coordinates: xd, yd, zd <br>
Current angles: φ, θ, ψ <br>
Let desired angles be zeros: φd = θd = ψd = 0 <br>

##### Calculate deltas
![Equations (29)](res/29.png)

##### Calculate commanded angles φc and θc and thrust T
![Equations (26)](res/26.png)

##### Calculate torques
![Equations (30)](res/30.png)

##### Calculate control inputs (angular velocities of engines)
![Equations (24)](res/24.png)



### References
* [Modelling and control of quadcopter](http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf)
* [Инженерный вестник](res/Gurianov.pdf)
* [ДИССЕРТАЦИЯ на соискание степени МАГИСТРА](http://elib.spbstu.ru/dl/2/v17-5857.pdf/download/v17-5857.pdf)
