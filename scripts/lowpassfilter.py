from numpy import array, zeros, dot

class lpf(object):
    def __init__(self, A, B, C, D):
        self.A = array(A)
        self.B = array(B)
        self.C = array(C)
        self.D = array(D)

    def input(self, signal):
        N = len(signal)
        self.u = signal
        self.x = zeros((N, 2))
        for i in range(N-1):
            self.x[i + 1] = dot(self.A, self.x[i]) + self.B * self.u[i]
        
        self.y = dot(self.C, self.x.T) + self.D * self.u

    def output(self):
        return self.y
