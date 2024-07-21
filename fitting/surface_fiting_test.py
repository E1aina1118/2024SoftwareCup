# coding=utf-8
from sympy import symbols, diff, lambdify  
import numpy as np  
  
# 定义变量  
x = symbols('x')  
  
# 定义多项式  
polynomial = 2*x**3 + 3*x**2 - 4*x + 5  
  
# 对多项式求导  
derivative = diff(polynomial, x)  
  
# 打印导数  
print("Derivative:", derivative)  
  
# 如果你还想要在某个具体的x值上计算导数的值  
# 可以使用lambdify将导数表达式转换为函数，然后传入x的值  
derivative_func = lambdify(x, derivative)  
x_value = 2  # 举例，x=2  
print("Derivative at x = 2:", derivative_func(x_value))  
  
# 如果你还想要绘制原多项式和它的导数  
import matplotlib.pyplot as plt  
  
# 生成x的数据点  
x_points = np.linspace(-5, 5, 400)  
  
# 计算原多项式和导数的值  
y_poly = lambdify(x, polynomial)(x_points)  
y_deriv = lambdify(x, derivative)(x_points)  
  
# 绘制图形  
plt.figure(figsize=(10, 5))  
plt.plot(x_points, y_poly, label='Polynomial')  
plt.plot(x_points, y_deriv, label='Derivative', linestyle='--')  
plt.legend()  
plt.show()