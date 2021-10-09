
def write_goal(goal_domino,domain,problem):
    s = "(picked d"+str(goal_domino)+" )"
    domain = domain.replace("###GOAL_HERE###",s)
    problem = problem.replace("###GOAL_HERE###",s)
    return domain,problem

def write_objects(n,problem):
    s = ""
    for i in range(n):
        s += "d"+str(i)+" "
    s += "- domino"
    problem = problem.replace("###DOMINOS_HERE###",s)
    return problem

def write_prob_effects(n,k,goal_domino,domain):
    s = "(:probabilistic " + "\n"
    for i in range(goal_domino - k, goal_domino+k+1):
        if i == goal_domino:
            continue
        p = int((100.0 / float(abs(goal_domino - i) + 1)) * 100) / 100.0
        s += str(p) + " (dropped d"+str(i)+")" + "\n"
    s += ")"
    domain = domain.replace("###PROBABILISTIC_EFFECTS_HERE###",s)
    return domain


if __name__ == "__main__":
    n = 15
    k = 2
    goal_domino = 4
    domain = open("domain_skeleton.ppddl","r").read()
    problem = open("problem_skeleton.ppddl","r").read()
    domain,problem = write_goal(goal_domino,domain,problem)
    domain = write_prob_effects(n,k,goal_domino,domain)
    problem = write_objects(n,problem)
    new_domain = open("domain.ppddl","w")
    new_problem = open("problem.ppddl","w")
    new_domain.write(domain)
    new_problem.write(problem)
    new_domain.close()
    new_problem.close()
