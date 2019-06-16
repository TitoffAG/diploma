import db
import plot
import utils
import numpy as np
from sympy import integrate, symbols
from scipy.optimize import fsolve
from scipy.integrate import odeint 
from scipy.interpolate import make_interp_spline, BSpline

# константы
alpha1 = 1.5
alpha3 = 1.5
end_time = 9
# интервал (1.5, 3.7) с углами в 50 градусов
# интервал (3.7, 5) с углами в 5 градусов
alpha2_start = 0.8
alpha2_end = 2.0
teta = 5
psi = 5
phi = 5
# начальные значения для p1, p2, p3; интервалы t и alpha3
init_approx = [1, 2, 3]
time = np.linspace(0, end_time, 101)
alpha_interval = np.linspace(alpha2_start, alpha2_end, 11)

lambda00, lambda01, lambda02, lambda03 = utils.transform_euler_angles_to_components_quaternion(
    teta, psi, phi)

lambdaT0 = -1.0
lambdaT1 = 0.0
lambdaT2 = 0.0
lambdaT3 = 0.0


def system_hof(vars, time, alpha):  # исходная система ДУ
    def system(vars, time):
        lambda0, lambda1, lambda2, lambda3, p1, p2, p3 = vars[
            0], vars[1], vars[2], vars[3], vars[4], vars[5], vars[6]

        omega1 = p1 / (4 * alpha1)
        omega2 = p2 / (4 * alpha)
        omega3 = p3 / (4 * alpha3)

        f1 = -0.5 * (lambda1 * omega1 + lambda2 * omega2 + lambda3 * omega3)
        f2 = 0.5 * (lambda0 * omega1 + lambda2 * omega3 - lambda3 * omega2)
        f3 = 0.5 * (lambda0 * omega2 + lambda3 * omega1 - lambda1 * omega3)
        f4 = 0.5 * (lambda0 * omega3 + lambda1 * omega2 - lambda2 * omega1)
        f5 = p2 * omega3 - omega2 * p3
        f6 = p1 * omega3 - omega1 * p3
        f7 = p1 * omega2 - omega1 * p2

        return f1, f2, f3, f4, f5, f6, f7

    return system


def calculate_ivp(vars, alpha):  # задача Коши
    p1, p2, p3 = vars[0], vars[1], vars[2]
    return odeint(system_hof(vars, time, alpha), [lambda00, lambda01, lambda02, lambda03, p1, p2, p3], time)


# функция F(p01, p02, p03) => вектор-невязок решений
def calculate_final_condition(vars):
    lambdaT_pribl0, lambdaT_pribl1, lambdaT_pribl2, lambdaT_pribl3 = vars[
        0], vars[1], vars[2], vars[3]
    lambdaT_pribl = np.array(
        [lambdaT_pribl0, lambdaT_pribl1, lambdaT_pribl2, lambdaT_pribl3])
    lambdaT = np.array([lambdaT0, lambdaT1, lambdaT2, lambdaT3])

    return utils.residual_calculation(lambdaT_pribl, lambdaT)


def calculate_bvp(vars, alpha):  # функция F(p01, p02, p03)
    def subordinate_solution_final_condition(vars):
        ivp_solution = calculate_ivp(vars, alpha)
        return calculate_final_condition(ivp_solution[-1])

    # F(p01, p02, p03) = (0, 0, 0)
    return fsolve(subordinate_solution_final_condition, vars)


def calculate_control(vars, alpha):
    p1, p2, p3 = vars[0], vars[1], vars[2]
    return [
        p1 / (4 * alpha1),
        p2 / (4 * alpha),
        p3 / (4 * alpha3)
    ]


def calculate_functional(vars, alpha):  # минимизируемый функционал
    control1, control2, control3 = vars[0], vars[1], vars[2]
    integrand_function = alpha1 * \
        (control1 ** 2) + alpha * (control2 ** 2) + alpha3 * (control3 ** 2)
    return integrate(integrand_function, (symbols('t'), 0, end_time))


def start():
    result_control = []
    result_functional = []
    for alpha in alpha_interval:
        # ищем аппроксимацию
        # подставляем начальное приближение
        bvp_solution = calculate_bvp(init_approx, alpha)
        print('[p1, p2, p3] = %s \n' %
              bvp_solution)  # подбор p1, p2, p3

        ivp_solution = calculate_ivp(bvp_solution, alpha)
        print('lambdaT: %s \n' %
              ivp_solution[-1])  # решение задачи Коши

        control = calculate_control(bvp_solution, alpha)
        print('Control: %s \n' %
              control)  # вычисление угловой скорости-управления
        result_control.append(control)

        functional = calculate_functional(control, alpha)
        print('Functional: %s \n' %
              functional)  # вычисление функционала
        result_functional.append(functional)

    result_control_bspline = make_interp_spline(alpha_interval, result_control, k=3)
    result_control_new = result_control_bspline(alpha_interval)
    result_functional_bspline = make_interp_spline(alpha_interval, result_functional, k=3)
    result_functional_new = result_functional_bspline(alpha_interval)
    # строим график изменения кватерниона во времени
    plot.draw_ivp_and_control(
        ivp_solution, result_control_new, time, alpha_interval)
    plot.draw_fuctional(result_functional_new, alpha_interval)

    results = bvp_solution.tolist(), ivp_solution[-1].tolist()

    ivp_and_control_charts = plot.open('resources/ivp_and_control.png')
    functional_chart = plot.open('resources/functional.png')
    charts = ivp_and_control_charts, functional_chart

    collection = db.connect() # подключаемся к серверу бд
    db.write(collection, results, charts) # делаем запись в коллекцию

    cursor = db.read(collection, {}) # поиск
    for record in cursor:
        print(record['data']) # читаем все записи из коллекции


if __name__ == '__main__':
    start()
