import math

# Solicitar al usuario el diámetro
diametro = float(input("Introduce el diámetro del círculo: "))

# Calcular el perímetro (circunferencia)
perimetro = math.pi * diametro

# Calcular el área
radio = diametro / 2
area = math.pi * (radio ** 2)

# Mostrar los resultados con dos decimales usando formato .2f
print(f"El perímetro del círculo es: {perimetro:.2f}")
print(f"El área del círculo es: {area:.2f}")
