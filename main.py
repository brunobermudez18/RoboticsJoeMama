import math


# Solicitar al usuario el diámetro
diametro = float(input("Introduce el diámetro del círculo: "))

# Calcular el perímetro (circunferencia)
perimetro = math.pi * diametro

# Calcular el área
radio = diametro / 2
area = math.pi * (radio ** 2)
# Calcular el volumen de una esfera
volumen = (4/3) * math.pi * (radio ** 3)

# Mostrar el resultado del volumen con dos decimales usando formato .2f
print(f"El volumen de la esfera es: {volumen:.2f}")
# Mostrar los resultados con dos decimales usando formato .2f
print(f"El perímetro del círculo es: {perimetro:.2f}")
print(f"El área del círculo es: {area:.2f}")
