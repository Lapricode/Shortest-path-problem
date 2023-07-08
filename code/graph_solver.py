import pymprog


def solve_IP_problem(A, b, c, decision_variables_names, path_name = None):
    model = pymprog.begin('Find shortest path')  # begin modelling the problem
    model.verbose(False)  # be analytic when analysing the model
    n, m = len(A), len(A[0])  # dimensions of matrix A
    # m is the number of decision variables and n is the number of constraints

    # create a list of n integer and non negative variables
    x_vars = []
    for i in range(m):
        x_vars.append(model.var(decision_variables_names[i], kind = int))
    for i in range(m):
        x_vars[i] <= 1  # the decision variables are bounded by 1 (they are binary)

    # create the constraints, y is the list of dual variables
    y_vars = [None] * n
    for i in range(n):
        y_vars[i] = sum(A[i][j] * x_vars[j] for j in range(m)) == b[i]

    # create the objective function and maximize it
    model.minimize(sum(c[i] * x_vars[i] for i in range(m)), "Cost function")

    # solve the model
    model.solve()
    if path_name != None:
        pymprog.save(mip = f"problem_solution_{path_name}.txt")  # save the problem's solution
        pymprog.save(clp = f"problem_model_{path_name}.txt")  # save the problem's model

    model.end()  # do away with the model

    return([x_vars[i].primal for i in range(m)])  # return the optimal values of the decision variables
