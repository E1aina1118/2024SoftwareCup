# coding=utf-8
from sympy import symbols, diff, lambdify  
import numpy as np  
  
# �������  
x = symbols('x')  
  
# �������ʽ  
polynomial = 2*x**3 + 3*x**2 - 4*x + 5  
  
# �Զ���ʽ��  
derivative = diff(polynomial, x)  
  
# ��ӡ����  
print("Derivative:", derivative)  
  
# ����㻹��Ҫ��ĳ�������xֵ�ϼ��㵼����ֵ  
# ����ʹ��lambdify���������ʽת��Ϊ������Ȼ����x��ֵ  
derivative_func = lambdify(x, derivative)  
x_value = 2  # ������x=2  
print("Derivative at x = 2:", derivative_func(x_value))  
  
# ����㻹��Ҫ����ԭ����ʽ�����ĵ���  
import matplotlib.pyplot as plt  
  
# ����x�����ݵ�  
x_points = np.linspace(-5, 5, 400)  
  
# ����ԭ����ʽ�͵�����ֵ  
y_poly = lambdify(x, polynomial)(x_points)  
y_deriv = lambdify(x, derivative)(x_points)  
  
# ����ͼ��  
plt.figure(figsize=(10, 5))  
plt.plot(x_points, y_poly, label='Polynomial')  
plt.plot(x_points, y_deriv, label='Derivative', linestyle='--')  
plt.legend()  
plt.show()