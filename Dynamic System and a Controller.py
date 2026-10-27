import matplotlib.pyplot as plt

# Dynamic System Parameters
m = 2
b = 10
k = 400

#PID Controller Parameters
Kp = 600.0
Ki = 50.0
Kd = 40.0

#Simulation settings
dt = 0.001
simulation_time = 5.0
steps = int(simulation_time / dt)

#Desired position
x_desired = 0.5

#Initial conditions
x = 1.0
v = 0.0
integral_error = 0.0
previous_error = x_desired - x

#Lists for plotting
time_values = []
position_values = []
forces_values = []

#Simulation loop
for i in range(steps):
    t = i * dt

    # Calculate error
    error = x_desired - x

    #PID Controller
    integral_error += error * dt
    derivative_error = (error - previous_error) / dt

    force = Kp * error + Ki * integral_error + Kd * derivative_error

    #Robot dynamics system
    acceleration = (force - b * v - k * x) / m

    #Update velocity and position
    v += acceleration * dt
    x += v * dt

    previous_error = error

    #Store results 
    time_values.append(t)
    position_values.append(x)
    forces_values.append(force)

#Plotting results
plt.figure()
plt.plot(time_values, position_values, label='Robot Position')
plt.axhline(y=x_desired, color='r', linestyle='--', label='Desired Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Robot Surgery System - PID Control')
plt.legend()
plt.grid(True)

#Plot controller force
plt.figure()
plt.plot(time_values, forces_values, label='Applied Force', color='g')
plt.xlabel('Time (s)')
plt.ylabel('Force')
plt.title('PID Controller Output')
plt.legend()
plt.grid(True)

plt.show()