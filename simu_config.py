# Sampling period (in s)
dt = 0.01

# Total simulation time (in s)
total_time = 10

# Plot states relations
plot_states = False

# Plot other wheel speed for comparison
plot_otherspeed = True

# System static gains (in tics/s/%)
# Broken robot
Kleft  = 100000
Kright = 200000
# Perfect robot
Kleft  = 100000
Kright = 100000
# Normal robot
Kleft  = 100000
Kright = 105000

# System time constants (in s)
# Broken robot
Tleft = 0.2
Tright = 0.1
# Perfect robot
Tleft = 0.150
Tright = 0.150
# Normal robot
Tleft = 0.155
Tright = 0.157

# Relation between wheels speeds difference and angular rate (in tics/rad)
diffticsperrad = 21772

# Relation between average wheels speed and curvilinear speed (in tics/mm)
ticspermeter = 278000

# Aleas in static gains (eg. 0.2 means K, + or - 0.2 * Kleft)
alea_Kleft = 0
alea_Kright = 0

# Aleas in time constants (same meaning)
alea_Tleft = 0
alea_Tright = 0
