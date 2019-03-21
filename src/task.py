import db
import plot
import utils
import numpy as np
from sympy import integrate, symbols
from scipy.optimize import fsolve
from scipy.integrate import odeint

# константы
alpha1 = 1.5
alpha2 = 2.5
alpha3 = 5
# задаем
end_time = 20
teta = 50
psi = 50
phi = 50
# начальные значения для p1, p2, p3 и интервалы времени
init_approx = [1, 2, 3]
time = np.linspace(0, end_time, 201)
alpha_interval = np.linspace(1.0, alpha3, 11)

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
        omega2 = p2 / (4 * alpha2)
        omega3 = p3 / (4 * alpha)

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
        p2 / (4 * alpha2),
        p3 / (4 * alpha)
    ]


def calculate_functional(vars, alpha):  # минимизирующий функционал
    control1, control2, control3 = vars[0], vars[1], vars[2]
    integrand_function = alpha1 * \
        (control1 ** 2) + alpha2 * (control2 ** 2) + alpha * (control3 ** 2)
    return integrate(integrand_function, (symbols('t'), 0, end_time))


def start():
    result = []
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

        functional = calculate_functional(control, alpha)
        print('Functional: %s \n' %
            functional)  # вычисление функционала
        
        result.append(functional)

    # строим график изменения кватерниона во времени
    plot.draw_ivp_and_control(ivp_solution, control, time)
    plot.draw_fuctional(result, alpha_interval)

    # results = bvp_solution.tolist(), ivp_solution[-1].tolist(), control, float(functional)
    # charts = plot.open('resources/img.png')

    # collection = db.connect() # подключаемся к серверу бд
    # db.write(collection, results, charts) # делаем запись в коллекцию

    # cursor = db.read(collection, {})
    # for record in cursor:
    #     print(record['data']) # читаем все записи из коллекции

    # plot.save('resources/img.png')


if __name__ == '__main__':
    start()
