
[diagram1]: ./misc_images/diagram1.png

** KINEMATIC ANALYSIS **



I diagrammed the Kuka arm as follows:

![alt text][diagram1]


Using this, I obtained the following *DH* table:

***i** | **α** | **a** | **d** | **Θ**
--- | --- | --- | --- | --- |
1 | 0 | 0 | 0.750 | Θ1
2 | -90 | 0.350 | 0 | Θ2 - 90
3 | 0 | 1.250 | 0 | Θ3
4 | -90 | 0.054 | 1.500 | Θ4
5 | 90 | 0 | 0 | Θ5
6 | -90 | 0 | 0 | Θ6
G | 0 | 0 | 0.303 | 0

I derived exact *a* and *d* values using the `kr210.urdf.xacro` file.

`s = {alpha0: 0,      a0:   0,      d1: 0.75,
    alpha1: -pi/2,  a1: 0.35,     d2: 0,      q2: q2-pi/2,
    alpha2: 0,      a2: 1.25,     d3: 0,
    alpha3: -pi/2,  a3: -0.054,   d4: 1.5,
    alpha4: pi/2,   a4:   0,      d5: 0,
    alpha5: -pi/2,  a5: 0,        d6: 0,
    alpha6: 0,      a6: 0,        d7: 0.303,  q7: 0}`
