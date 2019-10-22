# Lunar Hopper GitHub Repo

This is the repo for the Lunar Hopper on-board computer and control system.


## Test script usage
Tester file format:
*time (seconds) | command | arguments*

Tester file example:
```
0 start high
0.01 gyro 1 2
0.05 accel 1 2 3
0.1 throttle 50
0.27 press 90
1 abort high
5 disarm high
10 end
```

Test file example output:
```
-0.08030: ACT_PIN_X_POS reads 0
-0.08025: ACT_PIN_X_NEG reads 0
-0.08022: ACT_PIN_Y_POS reads 0
-0.06462: ACT_PIN_Y_NEG reads 0
-0.06457: SOL_PIN_VENT reads 0
-0.03106: SOL_PIN_PRES reads 0
-0.03098: SOL_PIN_OX reads 0
0.000371: 0 start high
0.012370: 0.01 gyro 1 2
0.085079: 0.05 accel 1 2 3
0.108033: 0.1 throttle 50
0.326400: 0.27 press 90
1.030022: 1 abort high
5.057830: 5 disarm high
10.06733: 10 end
10.10730: Test has concluded
```