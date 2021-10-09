#ifndef MDPLIB_BOUNDEDRTDP_H
#define MDPLIB_BOUNDEDRTDP_H

#include "Solver.h"

#include "../State.h"

namespace mlsolvers {

/**
 * An SSP solver using the Bounded RTDP algorithm.
 * http://machinelearning.wustl.edu/mlpapers/paper_files/icml2005_McMahanLG05.pdf
 */
class BoundedRTDPSolver : public Solver
{
private:

    mlcore::Problem* problem_;
    int maxTrials_;

    /* Criterion for stopping the algorithm
      (the difference between the bounds). */
    double epsilon_;

    /* Criterion for considering a state as well-known. */
    double tau_;

    /* Upper bounds on the state costs. */
    mlcore::StateDoubleMap upperBounds_;

    mlcore::StateActionMap lowerBoundGreedyPolicy_;

    /* Performs a single BRTDP trial. */
    void trial(mlcore::State* s);

    /* Performs a Bellman update for the state, changing its estimated cost
     * and the values of the bounds. */
    double bellmanUpdate(mlcore::State* s);

    /* Initializes the upper bound for the given state. */
    void initializeUpperBound(mlcore::State* s);

    /*
     * Samples a state biased according to the difference of its bounds.
     * Returns a nullptr if all successor states have "well-known" value.
     */
    mlcore::State* sampleBiased(mlcore::State* s, mlcore::Action* a);

public:
    /**
     * Creates a BRTDP solver for the given problem.
     *
     * @param problem The problem to be solved.
     * @param maxTrials The maximum number of trials to perform.
     * @param epsilon The error tolerance.
     */
    BoundedRTDPSolver(mlcore::Problem* problem,
                      double epsilon,
                      int maxTrials = 1000000);

    /**
     * Solves the associated problem using the Labeled RTDP algorithm.
     *
     * @param s0 The state to start the search at.
     */
    virtual mlcore::Action* solve(mlcore::State* s0);
};

}

#endif // MDPLIB_BOUNDEDRTDP_H
