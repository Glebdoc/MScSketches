import scipy.minimaze as opt 


def objective(x):
    """Objective function to minimize."""
    # Unpack the variables from the input array x
    main_diameter = x[0]
    small_diameter = x[1]

    
