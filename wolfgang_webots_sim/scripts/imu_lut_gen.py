
def noise_table(my_min, noise, iters, indent):
    print(indent * " ", end="")
    print("lookupTable [")
    for i in range(iters):
        val = my_min/2 ** i
        print(indent * " ", end="")
        print(f"  {val:f} {val:f} {noise/-val:f},")
    print(indent * " ", end="")
    print(f"  0 0 0")
    for i in range(iters):
        val = -my_min/2 ** (iters-i-1)
        print(indent * " ", end="")
        print(f"  {val:f} {val:f} {noise/val:f},")
    print(indent * " ", end="")
    print("]")

accel_min = -156.96
accel_noise =  0.03290374028
gyro_min = -34.90659
gyro_noise = 0.00106526458

noise_table(accel_min, accel_noise, 12, 12)
noise_table(gyro_min, gyro_noise, 12, 12)
