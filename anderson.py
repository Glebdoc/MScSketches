import numpy as np 

class AndersonAccelerator:
    def __init__(self, m=5):
        """
        Initialize the Anderson Acceleration method.
        
        Parameters:
        m (int): Number of previous iterates to use for acceleration.
        """
        self.m = m # Number of previous iterates to use
        self.Fs = []  # List to store function values
        self.Xs = []  # List to store previous iterates
    
    def apply(self, x, F_x):
        f = F_x - x
        self.Fs.append(f)
        self.Xs.append(x)
        if len(self.Fs) > self.m:
            self.Fs.pop(0)
            self.Xs.pop(0)

        if len(self.Fs) < 2:
            return F_x  # not enough history

        # Stack f and x differences
        dF = np.column_stack([f_i - self.Fs[-1] for f_i in self.Fs[:-1]])
        dX = np.column_stack([x_i - self.Xs[-1] for x_i in self.Xs[:-1]])

        try:
            gamma = np.linalg.lstsq(dF.T, f - self.Fs[-1], rcond=None)[0]
            dx = -dX @ gamma
            x_new = F_x + dx
        except np.linalg.LinAlgError:
            x_new = F_x  # fallback if solve fails

        return x_new