# main.py - 
from machine import Pin, I2C, UART
import time
from hamming74 import hamming74_encode, hamming74_decode  # ← Usar las funciones suministradas

# Configuración I2C con GP14 y GP15
i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)

# UART para osciloscopio
uart = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13))

# Dirección MPU6050
MPU6050_ADDR = 0x68

def setup_mpu6050():
    """Configurar el MPU6050"""
    i2c.writeto_mem(MPU6050_ADDR, 0x6B, b'\x00')  # Despertar
    time.sleep(0.2)

def scan_i2c():
    """Escanear dispositivos I2C"""
    devices = i2c.scan()
    print("Dispositivos I2C encontrados:", [hex(d) for d in devices])
    return devices

def read_accel():
    """Leer datos del acelerómetro (16 bits por eje)"""
    try:
        # Leer 6 bytes del registro 0x3B
        data = i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 6)
        # Convertir a valores de 16 bits
        accel_x = (data[0] << 8) | data[1]
        accel_y = (data[2] << 8) | data[3]
        accel_z = (data[4] << 8) | data[5]
        # Ajustar complemento a 2
        if accel_x > 32767: accel_x -= 65536
        if accel_y > 32767: accel_y -= 65536
        if accel_z > 32767: accel_z -= 65536
        return accel_x, accel_y, accel_z
    except Exception as e:
        print(f"Error leyendo acelerómetro: {e}")
        return 0, 0, 0

def split_16bit_to_nibbles(value):
    """Dividir valor de 16 bits en 4 nibbles de 4 bits"""
    value = abs(value) & 0xFFFF
    nibbles = []
    for i in range(4):
        shift = 4 * (3 - i)  # 12, 8, 4, 0
        nibble_val = (value >> shift) & 0x0F
        # [d3, d2, d1, d0] (MSB->LSB)
        bits = [
            (nibble_val >> 3) & 1,  # d3
            (nibble_val >> 2) & 1,  # d2
            (nibble_val >> 1) & 1,  # d1
            (nibble_val >> 0) & 1   # d0
        ]
        nibbles.append(bits)
    return nibbles

def encode_sample(sample_16bit):
    """Codificar muestra de 16 bits usando Hamming(7,4)"""
    print(f"Valor original: 0x{sample_16bit:04X} = {sample_16bit:016b}")
    # 1) Dividir en 4 nibbles
    nibbles = split_16bit_to_nibbles(sample_16bit)
    print("Nibbles obtenidos:")
    for i, nibble in enumerate(nibbles):
        nibble_val = nibble[0]*8 + nibble[1]*4 + nibble[2]*2 + nibble[3]
        print(f"  Nibble {i+1}: {nibble} = 0x{nibble_val:X}")
    # 2) Aplicar Hamming a cada nibble
    encoded_bits = []
    for i, nibble in enumerate(nibbles):
        encoded = hamming74_encode(nibble)
        encoded_bits.extend(encoded)
        nibble_val = nibble[0]*8 + nibble[1]*4 + nibble[2]*2 + nibble[3]
        print(f"  Nibble 0x{nibble_val:X} → Hamming: {encoded}")
    print(f"Bits codificados totales ({len(encoded_bits)}): {encoded_bits}")
    return encoded_bits

def bits_to_bytes(bits):
    """Convertir lista de bits a bytes para transmisión UART"""
    bytes_list = bytearray()
    for i in range(0, len(bits), 8):
        byte_bits = bits[i:i+8]
        while len(byte_bits) < 8:
            byte_bits.append(0)
        byte_val = 0
        for bit in byte_bits:
            byte_val = (byte_val << 1) | bit
        bytes_list.append(byte_val)
    return bytes_list

def test_hamming_functions():
    """Probar que las funciones Hamming funcionan correctamente"""
    print("=== PRUEBA DE FUNCIONES HAMMING ===")
    test_data = [1, 0, 1, 1]  # d3=1, d2=0, d1=1, d0=1
    print(f"Datos originales: {test_data}")
    encoded = hamming74_encode(test_data)
    print(f"Codificado: {encoded}")
    decoded, syndrome, corrected, corrected_code = hamming74_decode(encoded)
    print(f"Decodificado: {decoded}, Syndrome: {syndrome}, Corregido: {corrected}")
    # Probar con error
    encoded_error = encoded.copy()
    encoded_error[6] = 1 - encoded_error[6]  # Cambiar último bit
    decoded_err, syndrome_err, corrected_err, corrected_code_err = hamming74_decode(encoded_error)
    print(f"Con error: {decoded_err}, Syndrome: {syndrome_err}, Corregido: {corrected_err}")
    print("=== FIN PRUEBA ===\n")

def main():
    print("Iniciando Laboratorio Hamming MPU6050...")
    print("Pines I2C: SDA=GP14, SCL=GP15")
    print("UART TX: GP12 (para osciloscopio)\n")
    # Prueba Hamming
    test_hamming_functions()
    # Escanear I2C
    devices = scan_i2c()
    if MPU6050_ADDR not in devices:
        print("❌ MPU6050 no detectado. Verificar conexiones.")
        print(" - VCC → 3.3V")
        print(" - GND → GND")
        print(" - SDA → GP14 (Pin 19)")
        print(" - SCL → GP15 (Pin 20)")
        return
    print("✅ MPU6050 detectado correctamente!")
    setup_mpu6050()
    sample_count = 0
    while True:
        try:
            accel_x, accel_y, accel_z = read_accel()
            sample = accel_x
            encoded_bits = encode_sample(abs(sample) & 0xFFFF)
            encoded_bytes = bits_to_bytes(encoded_bits)
            uart.write(encoded_bytes)
            sample_count += 1
            print(f"Muestra #{sample_count} transmitida: {list(encoded_bytes)}")
            print("---" + "-"*40 + "---\n")
            time.sleep(2)
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()
