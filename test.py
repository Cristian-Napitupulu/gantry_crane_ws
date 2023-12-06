def pack_values(mode, pwm1, pwm2):
    # Ensure the values are within the specified ranges
    pwm1 = max(min(pwm1, 255), -255)
    pwm2 = max(min(pwm2, 255), -255)
    mode = max(min(mode, 255), 0)

    # Convert negative values to two's complement representation
    if pwm1 < 0:
        pwm1 = 0xFFF + pwm1 + 1
    if pwm2 < 0:
        pwm2 = 0xFFF + pwm2 + 1

    # Pack the values into a 32-bit integer
    packed_value = (mode & 0xFF) | ((pwm1 & 0xFFF) << 8) | ((pwm2 & 0xFFF) << 20)
    return packed_value

def unpack_values(packed_value):
    # Extract the values from the packed 32-bit integer
    mode = packed_value & 0xFF
    pwm1 = (packed_value >> 8) & 0xFFF
    pwm2 = (packed_value >> 20) & 0xFFF

    # Convert two's complement representation back to negative values
    if pwm1 & 0x800:
        pwm1 = pwm1 - 0xFFF - 1
    if pwm2 & 0x800:
        pwm2 = pwm2 - 0xFFF - 1

    return mode, pwm1, pwm2

# Example usage
mode_value = 1
pwm1_value = 200
pwm2_value = -100

packed_data = pack_values(mode_value, pwm1_value, pwm2_value)
print(f"Packed value: {hex(packed_data)}")

unpacked_mode, unpacked_pwm1, unpacked_pwm2 = unpack_values(packed_data)
print(f"Unpacked Mode: {unpacked_mode}, PWM1: {unpacked_pwm1}, PWM2: {unpacked_pwm2}")
