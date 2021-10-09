//
// Created by naman on 7/19/18.
//

#include "PolicyTree.h"
#include <vector>
#include <string>
#include <map>
#include <netinet/in.h>
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <typeinfo>
#include <unistd.h>


#include "include/ppddl/mini-gpt/domains.h"
#include "include/ppddl/mini-gpt/exceptions.h"
#include "include/ppddl/mini-gpt/formulas.h"
#include "include/ppddl/mini-gpt/problems.h"
#include "include/ppddl/mini-gpt/states.h"

#include "include/ppddl/PPDDLHeuristic.h"
#include "include/ppddl/PPDDLProblem.h"
#include "include/ppddl/PPDDLState.h"

#include "include/solvers/FLARESSolver.h"
#include "include/solvers/HDPSolver.h"
#include "include/solvers/LAOStarSolver.h"
#include "include/solvers/SoftFLARESSolver.h"
#include "include/solvers/Solver.h"
#include "include/solvers/SSiPPSolver.h"

#include "include/util/flags.h"

#include "include/State.h"

#define BUFFER_SIZE 65536


using namespace mdplib;
using namespace mlsolvers;
using namespace std;


extern int yyparse();
extern FILE* yyin;
string current_file;
int warning_level = 0;
int verbosity = 0;

static bool read_file( const char* name )
{
    yyin = fopen( name, "r" );
    if( yyin == NULL ) {
        cout << "parser:" << name <<
             ": " << strerror( errno ) << endl;
        return( false );
    }
    else {
        current_file = name;
        bool success;
        try {
            success = (yyparse() == 0);
        }
        catch( Exception exception ) {
            fclose( yyin );
            cout << exception << endl;
            return( false );
        }
        fclose( yyin );
        return( success );
    }
}


void initStringAtomMap(problem_t* problem,
                       unordered_map<string, ushort_t>& stringAtomMap)
{
    Domain dom = problem->domain();
    PredicateTable& preds = dom.predicates();
    TermTable& terms = problem->terms();
    for (auto const & atom : problem_t::atom_hash()) {
        ostringstream oss;
        atom.first->print(oss, preds, dom.functions(), terms);
        stringAtomMap[oss.str()] = atom.second;
    }
}


/* Returns the state corresponding to the given string. */
mlcore::State* getStatefromString(
        string stateString,
        mlppddl::PPDDLProblem* MLProblem,
        unordered_map<string, ushort_t>& stringAtomMap) {
    state_t *pState = new state_t();
    for (int i = 0; i < stateString.size(); i++) {
        if (stateString[i] == '(') {
            string atomString = "";
            int j;
            for (j = i; stateString[j] != ')'; j++)
                atomString += stateString[j];
            atomString += stateString[j];
            pState->add(stringAtomMap[atomString]);
            i = j;
        }
    }
    mlppddl::PPDDLState *newState = new mlppddl::PPDDLState(MLProblem);
    newState->setPState(*pState);
    return MLProblem->addState(newState);
}



class PolicyTreeNode {
public:
        mlcore::Action* action;
        mlcore::State* state;
        vector<PolicyTreeNode *>children;

        PolicyTreeNode(mlcore::Action* action ,
                        mlcore::State* state

        ){
            this->action = action;
            this->state = state;
        }
};


class PolicyTree {
public:
    PolicyTreeNode* root;

    void insertNode(PolicyTreeNode* parent, PolicyTreeNode* node){
        if (parent == nullptr)
            root = node;
        else
            parent->children.push_back(node);
    }
};


int main(int argc, char** argv){
    PolicyTree* tree = new PolicyTree;
    problem_t *problem = NULL;
    string file = "/data/ppddl/ippc2006/blocksworld/domProb.pddl";
    string prob = "bw_5_20405";
    if( !read_file( file.c_str() ) ) {
        cout <<
             "<main>: ERROR: couldn't read problem file. `" << file << endl;
        exit(-1);
    }

    problem = (problem_t*) problem_t::find( prob.c_str() );
    if( !problem ) {
        cout << "<main>: ERROR: problem `" << prob <<
             "' is not defined in file. '" << file << "'" << endl;
        exit(-1);
    }
//
//    /* Initializing problem. */
    mlppddl::PPDDLProblem* MLProblem = new mlppddl::PPDDLProblem(problem);
//    mlppddl::PPDDLHeuristic* heuristic =
//            new mlppddl::PPDDLHeuristic(MLProblem, mlppddl::FF);
//    MLProblem->setHeuristic(heuristic);
//
//    unordered_map<string, ushort_t> stringAtomMap;
//    initStringAtomMap(problem, stringAtomMap);
//



    return 0;
}





