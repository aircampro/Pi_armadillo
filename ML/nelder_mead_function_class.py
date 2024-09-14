import numpy as np

class griewank():
    def __init__(self):
        print("this is Griewank function.")
        self.boundaries = np.array([-600, 600])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        w = np.array([1.0 / np.sqrt(i + 1) for i in range(len(x))])
        t1 = 1
        t2 = 1.0 / 4000.0 * np.sum(x ** 2)
        t3 = - np.prod(np.cos(x * w))
        return t1 + t2 + t3
		
class perm():
    def __init__(self):
        print("this is Perm function.")
        self.boundaries = np.array([-1, 1])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        val = 0
        for j in range(len(x)):
            v = 0

            for i in range(len(x)):
                v += (i + 2) * (x[i] ** (j + 1) - ( ( 1 / (i + 1) ) ** (j + 1) ) )
            val += v ** 2

        return val
		
class rastrigin():
    def __init__(self):
        print("this is Rastrigin function")
        self.boundaries = np.array([-5.12, 5.12])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        t1 = 10 * len(x)
        t2 = np.sum(x ** 2)
        t3 = - 10 * np.sum(np.cos(2 * np.pi * x))
        return t1 + t2 + t3
		
class schwefel():
    def __init__(self):
        print("this is Schwefel function")
        self.boundaries = np.array([-500, 500])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {} times n_dim".format(-418.9829))

    def f(self, x):
        return - np.sum(x * np.sin( np.sqrt( np.abs(x) ) ) )
		
class xin_she():
    def __init__(self):
        print("this is Xin-She Yang function.")
        self.boundaries = np.array([-2 * np.pi, 2 * np.pi])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        t1 = np.sum( np.abs(x) )
        e1 = - np.sum( np.sin(x ** 2) )
        t2 = np.exp(e1)
        return t1 * t2
		
class zakharov():
    def __init__(self):
        print("this is Zakharov function.")
        self.boundaries = np.array([-100, 100])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        t1 = np.sum(x)
        w = np.array([ i + 1 for i in range(len(x))])
        wx = np.dot(w, x)
        t2 = 0.5 ** 2 * wx ** 2
        t3 = 0.5 ** 4 * wx ** 4
        return t1 + t2 + t3
		
class rotated_hyper_ellipsoid():
    def __init__(self):
        print("this is rotated hyper ellipsoid function.")
        self.boundaries = np.array([-65.536, 65.536])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        val = 0
        for i in range(len(x)):
            val += np.sum(x[:i + 1] ** 2)
        return val
		
class trid():
    def __init__(self, n_dim = 10):
        print("this is rotated trid function.")
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        n_dim = len(x)
        self.boundaries = np.array([- n_dim ** 2, n_dim ** 2])

        t1 = np.sum( (x - 1) ** 2 )
        t2 = - np.sum( x[1:n_dim] * x[0:n_dim - 1] )

        return t1 + t2
		
class dixon_price():
    def __init__(self):
        print("this is rotated dixon price function.")
        self.boundaries = np.array([- 10, 10])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        n_dim = len(x)
        c = np.array([i + 2 for i in range(n_dim - 1)])
        t1 = (x[0] - 1) ** 2
        t2 = np.sum( c * (2 * x[1:n_dim] ** 2 - x[0:n_dim - 1] ) ** 2 )

        return t1 + t2
		
class levy():
    def __init__(self):
        print("this is levy price function.")
        self.boundaries = np.array([- 10, 10])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        w = np.array(1. + (x - 1) / 4.)
        t1 = np.sin(np.pi * w[0]) ** 2
        t2 = (w[-1] - 1) ** 2 * (1 + np.sin(2 * np.pi * w[-1]) ** 2)
        t3 = np.sum( (w[:-1] - 1) ** 2 * (1 + 10 * np.sin(np.pi * w[:-1] + 1) ** 2) )

        return t1 + t2 + t3
		
class different_power():
    def __init__(self):
        print("this is Sum of different power function.")
        self.boundaries = np.array([-1, 1])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        val = 0
        for i, v in enumerate(x):
            val += np.abs(v) ** (i + 2)
        return val
		
class weighted_sphere():
    def __init__(self):
        print("this is Weighted Sphere function or hyper ellipsodic function.")
        self.boundaries = np.array([-5.12, 5.12])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        val = np.array([ (i + 1) * xi ** 2 for i, xi in enumerate(x)])
        return np.sum(val)
		
class k_tablet():
    def __init__(self):
        print("this is k-tablet function.")
        self.boundaries = np.array([-5.12, 5.12])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        k = int(np.ceil(len(x) / 4.0))
        t1 = np.sum(x[:k] ** 2)
        t2 = 100 ** 2 * np.sum(x[k:] ** 2)
        return t1 + t2
		
class styblinski():
    def __init__(self):
        print("this is Styblinski-Tang function.")
        self.boundaries = np.array([-5, 4])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {} times n_dim".format(-39.166165))

    def f(self, x):
        t1 = np.sum(x ** 4)
        t2 = - 16 * np.sum(x ** 2)
        t3 = 5 * np.sum(x)
        return 0.5 * (t1 + t2 + t3)
		
class rosenbrock():
    def __init__(self):
        print("this is Rosenbrock function.")
        self.boundaries = np.array([-5, 5])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        val = 0
        for i in range(0, len(x) - 1):
            t1 = 100 * (x[i + 1] - x[i] ** 2) ** 2
            t2 = (x[i] - 1) ** 2
            val += t1 + t2
        return val
		
class sphere():
    def __init__(self):
        print("this is Sphere function.")
        self.boundaries = np.array([-100, 100])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        return np.sum(x ** 2)
		
class ackley():
    def __init__(self):
        print("this is Ackley function.")
        self.boundaries = np.array([-32.768, 32.768])
        print("boundary is {}".format(self.boundaries))
        print("minimum is {}".format(0))

    def f(self, x):
        t1 = 20
        t2 = - 20 * np.exp(- 0.2 * np.sqrt(1.0 / len(x) * np.sum(x ** 2)))
        t3 = np.e
        t4 = - np.exp(1.0 / len(x) * np.sum(np.cos(2 * np.pi * x)))
        return t1 + t2 + t3 + t4
		
