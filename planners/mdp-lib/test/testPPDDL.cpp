#include <sstream>
#include <typeinfo>

#include "../include/ppddl/mini-gpt/states.h"
#include "../include/ppddl/mini-gpt/problems.h"
#include "../include/ppddl/mini-gpt/domains.h"
#include "../include/ppddl/mini-gpt/states.h"
#include "../include/ppddl/mini-gpt/exceptions.h"

#include "../include/State.h"
#include "../include/ppddl/PPDDLProblem.h"
#include "../include/ppddl/PPDDLHeuristic.h"

#include "../include/solvers/Solver.h"
#include "../include/solvers/SoftFLARESSolver.h"
#include "../include/solvers/HMinHeuristic.h"
#include "../include/solvers/LRTDPSolver.h"
#include "../include/solvers/LAOStarSolver.h"
#include "../include/solvers/UCTSolver.h"

#include "../include/util/flags.h"

#include <queue>
#include <fstream>

using namespace mdplib;
using namespace mlsolvers;
using namespace std;

extern int yyparse();
extern FILE* yyin;
string current_file;
int warning_level = 0;
int verbosity = 1000;
bool using_soft_flares = false;

mlppddl::PPDDLProblem* MLProblem;

class PolicyTreeNode {
public:
    mlcore::Action* action;
    mlcore::State* state;
    vector<PolicyTreeNode *>children;
    float prob;
    int level;

    PolicyTreeNode(mlcore::Action* action ,
                   mlcore::State* state ,
                   float prob,
                   int level

    ){
        this->action = action;
        this->state = state;
        this->prob = prob;
        this->level = level;
    }
};

class PolicyTree {
public:
    PolicyTreeNode* root;

    void insertNode(PolicyTreeNode* node, PolicyTreeNode* parent){
        if (parent == nullptr)
            root = node;
        else
            parent->children.push_back(node);
    }
};




class QueueNode {
public:
    PolicyTreeNode* dataNode;
    QueueNode* next = nullptr;
    QueueNode(PolicyTreeNode* data) {
        this->dataNode = data;
    }

};

class PrintTreeQueueNode{
public:
    PolicyTreeNode* tnode = nullptr;
    PrintTreeQueueNode* next = nullptr;

    PrintTreeQueueNode(PolicyTreeNode* data) {
        this->tnode = data;
    }

};

class PrintTreeQueue {

public:
    PrintTreeQueueNode* head = nullptr;
    PrintTreeQueueNode* tail = nullptr;

    void insert(PolicyTreeNode* tnode){
        PrintTreeQueueNode* temp = new PrintTreeQueueNode(tnode);
        if (this->head == nullptr){
            head = temp;
            tail = temp;
        }
        else{
            tail->next = temp;
            tail = temp;
        }
    }

    PrintTreeQueueNode* remove(){
        PrintTreeQueueNode* temp = this->head;
        this->head = this->head->next;
        return temp;
    }

    bool isEmpty(){
        return head == nullptr;
    }
};


void printTree(PolicyTree* tree,float cost,string output_file_name ) {
//    PrintTreeQueue* pqueue = new PrintTreeQueue();
//    PolicyTreeNode* rootNode = tree->root;
//    pqueue->insert(rootNode);
//    pqueue->insert(nullptr);
//    PrintTreeQueueNode* currentNode;
//    int level = 0;
//    cout << "Level : 0 : " << endl;
//    while(!pqueue->isEmpty()){
//        currentNode = pqueue->remove();
//        if(currentNode->tnode == nullptr){
//            cout << "Level : "  << ++level << endl;
//            pqueue->insert(nullptr);
//        }
//        else{
//            cout << "State : " << currentNode->tnode->state << " Action : " << currentNode->tnode->action << endl;
//            for (PolicyTreeNode* ptn : currentNode->tnode->children) {
//                pqueue->insert(ptn);
//            }
//        }
//    }

//    int level = 0;
//    std::queue<PolicyTreeNode* > pqueue;
//    pqueue.push(new PolicyTreeNode(nullptr, nullptr));
//    pqueue.push(tree->root);
//    PolicyTreeNode* temp;
//    while(!pqueue.empty()){
//        temp = pqueue.front();
//        pqueue.pop();
//        if(temp->state == nullptr) {
//            cout << "Level " << level << " : " << endl;
//            level++;
//            pqueue.push(new PolicyTreeNode(nullptr, nullptr));
//        }
//        else {
//            cout << "State : "<<temp->state << endl;
//            cout << "Action : "<< temp->action << endl;
//            for (PolicyTreeNode* ptn :  temp->children) {
//                pqueue.push(ptn);
//            }
//        }
//    }

    ofstream outfile;
    cout << "Printing in File" << endl;
    outfile.open(output_file_name);
    outfile << "Digraph G {" << endl;
    outfile << "size = \"500,500\";" << endl;

    typedef std::pair<PolicyTreeNode* , string> Queuenode;

    std::queue<Queuenode > q;

    PolicyTreeNode* temp;

    Queuenode r;
    r.first = tree->root;
    int i = 0;
    r.second = to_string(i);
    outfile << to_string(i)  << " [ label = \" " << tree->root->state << " :: " << tree->root->action << " :: " << tree->root->prob << "\" ]; " << endl;


    string label;
    i++;
    q.push(r);
    string extra;
    while(!q.empty()){
        temp = q.front().first;
        label = q.front().second;
        q.pop();
        for (PolicyTreeNode* p : temp->children) {
            //outfile << "\" State : " << temp->state << " \" -> " << " \" Action : " << temp->action << " \" ->" << "\"State : " << p->state << " \";" <<endl;

            if (MLProblem->goal(p->state))
                extra = "[shape=box,style=filled,color=\".7 .3 1.0\"]";
            else
                extra = "";
            if ( p->action == nullptr) {

                outfile << to_string(i) << " [ label = \" " << p->state << " :: " << "STOP" << " :: " << p->prob << "\"  ];" << endl;
                outfile << label << "->" << to_string(i) << " " << extra << ";" << endl;
                Queuenode t;

                t.first = p;
                t.second = to_string(i);
                i++;
            }
                //outfile << "\"" << temp->state <<  " : " << temp->action << "\"" << "->" << "\"" << p->state <<  " : "  << "STOP" << "\"" << extra << ";" << endl;
            else {

                outfile << to_string(i) << " [ label = \" " << p->state << " :: " << p->action << " :: " << p->prob << "\" ]; " << endl;
                outfile << label << "->" << to_string(i) << " " << extra << ";" << endl;
                Queuenode t;

                t.first = p;
                t.second = to_string(i);
                i++;
                q.push(t);
            }
            //outfile << "\"" <<temp->state<<"\"" << "->" << "\"" << temp->action << "\"" << "->" << "\"" << p->state << "\""  << endl
        }
    }

    outfile << "}" <<endl;
    outfile << "# Cost: " << cost << endl;
    outfile.close();
    cout << "Done";
}




/* Parses the given file, and returns true on success. */
static bool read_file( const char* name )
{
    yyin = fopen( name, "r" );
    if( yyin == NULL ) {
        std::cout << "parser:" << name <<
            ": " << strerror( errno ) << std::endl;
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
            std::cout << exception << std::endl;
            return( false );
        }
        fclose( yyin );
        return( success );
    }
}

bool mustReplan(Solver* solver,
                mlcore::State* current_state,
                mlcore::Action* current_action) {
    if (current_action == nullptr)
        return true;
    if (using_soft_flares) {
        return !static_cast<SoftFLARESSolver*>(solver)->
                    labeledSolved(current_state);
    }
}

int main(int argc, char *args[])
{
    std::string file;
    std::string prob;
    std::string output_file_name;
    problem_t *problem = NULL;
    std::pair<state_t *,Rational> *initial = NULL;

    if (argc < 2) {
        std::cout << "Usage: testPPDDL [file] [problem]\n";
        return -1;
    }

    file = args[1];
    prob = args[2];
    int horizon = atoi(args[3]);
    output_file_name = args[4];

    if( !read_file( file.c_str() ) ) {
        std::cout <<
            "<main>: ERROR: couldn't read problem file `" << file << std::endl;
        return( -1 );
    }
    problem = (problem_t*) problem_t::find( prob.c_str() );
    if( !problem ) {
        std::cout << "<main>: ERROR: problem `" << prob <<
            "' is not defined in file '" << file << "'" << std::endl;
        return( -1 );
    }

    /* Initializing problem */
    MLProblem = new mlppddl::PPDDLProblem(problem);
    mlppddl::PPDDLHeuristic* heuristic =
//        new mlppddl::PPDDLHeuristic(MLProblem, mlppddl::atomMin1Forward);
//        new mlppddl::PPDDLHeuristic(MLProblem, mlppddl::atomMinMForward);
        new mlppddl::PPDDLHeuristic(MLProblem, mlppddl::FF);
    MLProblem->setHeuristic(heuristic);

    cout << "HEURISTIC s0: " << MLProblem->initialState()->cost() << endl;

    int ntrials = 50000000;
    if (argc > 3) {
        ntrials = atoi(args[3]);
    }

    cout << "INITIAL: " << MLProblem->initialState() << " ";
    Solver* solver;
    register_flags(argc, args);
    if (flag_is_registered("algorithm") &&
            flag_value("algorithm") == "soft-flares") {
        using_soft_flares = true;
        double depth = 4;
        double alpha = 0.10;
        double tol = 1.0e-3;
        int trials = 1000;
        if (flag_is_registered_with_value("depth"))
            depth = stoi(flag_value("depth"));
        TransitionModifierFunction mod_func = kLogistic;
        DistanceFunction dist_func = kStepDist;
        if (flag_is_registered_with_value("alpha"))
            alpha = stof(flag_value("alpha"));
        // Distance functions
        if (flag_is_registered("dist")) {
            string dist_str = flag_value("dist");
            if (dist_str == "traj") {
                dist_func = kTrajProb;
            } else if (dist_str == "plaus") {
                dist_func = kPlaus;
            } else if (dist_str == "depth") {
                dist_func = kStepDist;
            } else {
                cerr << "Error: unknown distance function." << endl;
                exit(0);
            }
        }
        // Labeling functions
        if (flag_is_registered("labelf")) {
            string labelf_str = flag_value("labelf");
            if (labelf_str == "exp") {
                mod_func = kExponential;
            } else if (labelf_str == "step") {
                mod_func = kStep;
            } else if (labelf_str == "linear") {
                mod_func = kLinear;
            } else if (labelf_str == "logistic") {
                mod_func = kLogistic;
            } else {
                cerr << "Error: unknown labeling function." << endl;
                exit(0);
            }
        }
        solver = new SoftFLARESSolver(
            MLProblem, trials, tol, depth, mod_func, dist_func, alpha);
        static_cast<SoftFLARESSolver*>(solver)->maxPlanningTime(1000);
    } else {
//        solver = new LRTDPSolver(MLProblem, ntrials, 0.0001);
//        solver->maxPlanningTime(1000);
            solver = new LAOStarSolver(MLProblem);
    }


    mdplib_debug = true;
    solver->solve(MLProblem->initialState());

//    cout << MLProblem->initialState()->cost() << endl;
//
//
//    int nsims = argc > 4 ? atoi(args[4]) : 1;
//    int verbosity = argc > 5 ? atoi(args[5]) : 10000;
//
//    int totalSuccess = 0;
//    double expectedCost = 0.0;
//    for (int i = 0; i < nsims; i++) {
//        mlcore::State* tmp = MLProblem->initialState();
//        double cost = 0.0;
//        while (true) {
//            mlcore::Action* a = tmp->bestAction();
//
//            if (verbosity > 100)
//                cout << tmp << " " << tmp->cost() << endl;
//
//            if (MLProblem->goal(tmp)) {
//                if (verbosity > 1)
//                    cout << "GOAL :-)" << endl;
//                expectedCost += cost;
//                totalSuccess++;
//                break;
//            }
//            if (mustReplan(solver, tmp, a)) {
//                if (verbosity > 1)
//                    cout << "REPLANNING..." << endl;
//                solver->solve(tmp);
//                a = tmp->bestAction();
//                if (tmp->deadEnd() || a == nullptr) {
//                    if (verbosity > 100)
//                      cout << "DEAD END!! giving up :-( " << endl;
//                    break;
//                }
//            }
//            cost += MLProblem->cost(tmp, a);
//
//            if (cost > mdplib::dead_end_cost
//                    || tmp->cost() >= mdplib::dead_end_cost) {
//                cout << "Too long... giving up " << endl;
//                break;
//            }
//
//            if (verbosity > 100)
//                cout << tmp->bestAction() << endl;
//            tmp = mlsolvers::randomSuccessor(MLProblem, tmp, a);
//        }
//    }
//    cout << "Expected Cost: " << expectedCost / totalSuccess << endl;
//    cout << "Total Successes " << totalSuccess << "/" << nsims << endl;

    mlcore::Action* action = nullptr;
    string actionDescription;
    mlcore::State* currentState = MLProblem->initialState();
    action = currentState->bestAction();

    PolicyTreeNode* temp = new PolicyTreeNode(action,currentState,1.0,1);
    PolicyTree* tree = new PolicyTree();
    tree->insertNode(temp, nullptr);

    float tempProb;
    int tempLevel;

    mlcore::State* tempState = nullptr;
    mlcore::Action* tempAction = nullptr;
    std::queue<PolicyTreeNode* > q;
    q.push(temp);
    float cost = 0;
    int count;
    int goal_reached = 0;

    while(!q.empty()) {
        temp = q.front();
        q.pop();
        tempAction = temp->action;
        tempState = temp->state;
        tempProb = temp->prob;
        tempLevel = temp->level;
        cout << "Number of Elements in Queue :"  << q.size() << endl;
        cost += tempProb * MLProblem->cost(tempState,tempAction);
        if (tempLevel <= horizon || goal_reached == 0){
            for(mlcore::Successor& su : MLProblem->transition(tempState,tempAction)){
                action = su.su_state->bestAction();
                count = 0;
                cout << "Action : " << action << endl;
                ostringstream oss;
                oss << action;
                actionDescription = oss.str();
                PolicyTreeNode* temp2;
                if(actionDescription == "(done)") {
                    temp2 = new PolicyTreeNode(action,su.su_state,tempProb * su.su_prob,tempLevel++);
                    tree->insertNode(temp2,temp);
                    mlcore::State* nextState = mlsolvers::randomSuccessor(MLProblem,su.su_state,action);
                    if (MLProblem->goal(nextState)) {
                        cout << endl << "*********GOAL REACHED**********" << endl;
                        goal_reached = 1;
                    }
                    else
                        cout << endl << "**** NOT GOAL ***** BUT NOT PUSHED *******" << endl;
                    PolicyTreeNode* temp3 = new PolicyTreeNode(nullptr,nextState,tempProb * su.su_prob,tempLevel++);
                    tree->insertNode(temp3,temp2);
                }
                else {
                    temp2 = new PolicyTreeNode(action,su.su_state,tempProb * su.su_prob,tempLevel++);
                    tree->insertNode(temp2,temp);
                    q.push(temp2);
                }
            }
        }

    }
//    cout << "Expected Cost: " << cost << endl;
    printTree(tree,cost,output_file_name);



    state_t::finalize();
    problem_t::unregister_use(problem);
    problem_t::clear();

    delete heuristic;
    delete solver;
}
