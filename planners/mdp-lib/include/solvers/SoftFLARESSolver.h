#ifndef MDPLIB_SoftFLARESSolver_H
#define MDPLIB_SoftFLARESSolver_H

#include "Solver.h"

#include "../Heuristic.h"
#include "../Problem.h"
#include "../State.h"


namespace mlsolvers {

enum TransitionModifierFunction {
    kExponential,
    kLogistic,
    kLinear,
    kStep
};

enum DistanceFunction {
    kStepDist,
    kTrajProb,
    kPlaus
};

/**
 * A SSP solver using the Soft-FLARES algorithm.
 */
class SoftFLARESSolver : public Solver {
private:
    /* ********************************************************************* *
                                    Variables
    /** ********************************************************************* */
    /* The problem to solve. */
    mlcore::Problem* problem_;

    /* The maximum number of trials. */
    int maxTrials_;

    /* If optimal is true, the algorithm will be run until the initial state
     * is marked as solved. */
    bool optimal_;

    /* The error tolerance */
    double epsilon_;

    /* The max depth for the checkSolved procedure */
    double horizon_;

    /* Maximum planning time in milliseconds. */
    int maxTime_;

    /*
     * Represents the desired reduction in the transition function at
     * distance 0. For example, if [alpha_] = 0.99, then a state s' that has
     * 0 steps of successors (i.e., only itself) with residual < [epsilon_] will
     * have a probability of being sampled equal to 0.99 * T(s,a,s').
     */
    double alpha_;

    /*
     * Represents the desired reduction in the transition function at the
     * horizon. For example, if [beta_] = 0.01, then a state s' that has
     * [horizon_] steps of successors with residual < [epsilon_] will
     * have a probability of being sampled equal to 0.01 * T(s,a,s').
     */
    double beta_;

    /*
     * Modifies the scoring function to achieve the desired [alpha_].
     */
    double tau_;

    /*
     * If true, the depth of states will be the log probability of reaching
     * the state. Otherwise, it is the number of steps.
     */
    bool useProbsForDepth_;

    /*
     * The function used to modify the transition function from the residual
     * distance estimates.
     */
    TransitionModifierFunction modifierFunction_;

    /*
     * The distance function to use for labeling.
     */
    DistanceFunction distanceFunction_;

    /* Stores the result of modifier function for depths from 0-[horizon_].*/
    std::vector<double> modifierCache_;

    /* If true, distances will be obtained from the [modifierCache_].*/
    bool useCache_;

    /* The time at which planning began. */
    std::chrono::time_point<std::chrono::high_resolution_clock> beginTime_;

    /* ********************************************************************* *
                                    Methods
    /* ********************************************************************* */
    /*
     * Samples a successor biased towards state closer to higher residual
     * error.
     */
    mlcore::State* sampleSuccessor(mlcore::State* s, mlcore::Action* a);

    /* Computes the distances to states with high residuals. */
    void computeResidualDistances(mlcore::State* s);

    /* Performs a single trial */
    void trial(mlcore::State* s);

    /*
     * Computes the depth of the given successor state for the given depth of
     * its parent state.
     */
    double computeNewDepth(mlcore::Successor& su, double depth);

    /*
     * Computes the "probability" that the state is not labeled.
     * Thus, The sampling probability will be
     *   T'(s,a,s') =
     *      T(s,a,s') * [computeProbUnlabeled](s'->residualDistance()).
     */
    double computeProbUnlabeled(mlcore::State* s);

    /*
     * Computes the probability that the state is not labeled, given a distance.
     * The sampling probability will be
     *   T'(s,a,s') = T(s,a,s') * [computeProbUnlabeled](distance)
     * The function to be used to compute this probability is
     * controlled using [modifierFunction_].
     */
    double computeProbUnlabeled(double distance);

    /*
     * Computes the unnormalized transition function using an exponential
     * decaying modifier and stores the result in the [scores] vector.
     * [totalScore] is the sum of all the scores, needed to normalize the new
     * distribution.
     */
    mlcore::State* computeScores(mlcore::State* s,
                                 mlcore::Action* a,
                                 std::vector<double>& scores,
                                 double& totalScore);


    /* Returns true if there should be more trials, false otherwise. */
    bool moreTrials(
        mlcore::State* s,
        int trialsSoFar,
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime);

                                                                                int cnt_samples_ = 0;
                                                                                long int total_time_samples_ = 0;
                                                                                int cnt_check_ = 0;
                                                                                long int total_time_check_ = 0;

public:
    /**
     * Creates a Soft-FLARES solver for the given problem.
     *
     * @param problem The problem to be solved.
     * @param maxTrials The maximum number of trials to perform.
     * @param epsilon The error tolerance.
     * @param horizon_ The maximum depth for checkSolved.
     * @param modifierFunction The function to use to modify the transition.
     * @param distanceFunction The distance function to use for labeling.
     * @param alpha Desired vanishing level.
     * @param useProbsForDepth If true, uses trajectory probabilities as
     *        the depth (instead of the number of steps).
     * @param optimal If true, runs until optimality.
     * @param maxTime The maximum time allowed for planning.
     */
    SoftFLARESSolver(mlcore::Problem* problem,
                     int maxTrials,
                     double epsilon,
                     double horizon_,
                     TransitionModifierFunction modifierFunction,
                     DistanceFunction distanceFunction,
                     double alpha = 0.01,
                     bool useProbsForDepth = false,
                     bool optimal = false,
                     int maxTime = -1);

    /**
     * Solves the associated problem using the Soft-FLARES algorithm.
     *
     * @param s0 The state to start the search at.
     */
    virtual mlcore::Action* solve(mlcore::State* s0);

    /** Returns the last estimate of the epsilon-distance of a state. */
    double lowResidualDistance( mlcore::State* s) const {
        return s->residualDistance();
    }

    double horizon() const { return horizon_; }

    /** Checks if the state has already been labeled as solved. */
    bool labeledSolved(mlcore::State* s);

    /**
     * Sets the maximum planning time allowed to the algorithm (milliseconds).
     */
    virtual void maxPlanningTime(time_t theTime) { maxTime_ = theTime; }

    /**
     * Sets the maximum number of trials allowed to the algorithm.
     */
    virtual void maxTrials(time_t theTrials) { maxTrials_  = theTrials; }

};

}

#endif // MDPLIB_SoftFLARESSolver_H
