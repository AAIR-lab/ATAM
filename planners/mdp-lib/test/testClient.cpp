#include <map>
#include <netdb.h>
#include <netinet/in.h>
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <typeinfo>
#include <unistd.h>

#include "../include/ppddl/mini-gpt/domains.h"
#include "../include/ppddl/mini-gpt/exceptions.h"
#include "../include/ppddl/mini-gpt/formulas.h"
#include "../include/ppddl/mini-gpt/problems.h"
#include "../include/ppddl/mini-gpt/states.h"

#include "../include/ppddl/PPDDLHeuristic.h"
#include "../include/ppddl/PPDDLProblem.h"
#include "../include/ppddl/PPDDLState.h"

#include "../include/solvers/Solver.h"

#include "../include/State.h"
#include <queue>
#include <fstream>

#define BUFFER_SIZE 6553600



using namespace std;
using namespace mlsolvers;

extern int yyparse();
extern FILE* yyin;
string current_file;
int warning_level = 0;
int verbosity = 0;


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

//class Queue {
//
//public:
//    int currentNodeNumber = 0 ,lastNodeNumber = 0;
//    QueueNode* currentNode = nullptr;
//    QueueNode* lastNode = nullptr;
//
//    void insert(PolicyTreeNode* tnode) {
//        QueueNode* node = new QueueNode(tnode);
//        if (currentNode == nullptr) {
//            currentNode = node;
//            lastNode = node;
//            currentNodeNumber++;
//            lastNodeNumber++;
//        }
//        else{
//            lastNode->next = node;
//            lastNode = node;
//            lastNodeNumber++;
//        }
//    }
//
//    PolicyTreeNode* remove(){
//        PolicyTreeNode* tnode = currentNode->dataNode;
//        currentNode = currentNode->next;
//        currentNodeNumber++;
//        return tnode;
//    }
//
//    bool isEmpty(){
//        return currentNode == nullptr;
//    }
//
//    int numberOfElements() {
//        return lastNodeNumber - currentNodeNumber + 1;
//    }
//
//};
//



/* Parses the given file, and returns true on success. */
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


/* Initializes the map from strings (describing atoms) to atoms ids. */
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
string getAtomsString(string stateString,
                      unordered_map<string, ushort_t>& stringAtomMap)
{
    string atomsString = "";
    for (int i = 0; i < stateString.size(); i++) {
        if (stateString[i] == '(') {
            int j;
            for (j = i; stateString[j] != ')'; j++)
                atomsString += stateString[j];
            atomsString += stateString[j];
            atomsString +=  " ";
            i = j;
        }
    }
    return atomsString;
}


/* Simulates receiving an action description from the client. */
string
getActionFromServer(int sockfd,
                    mlcore::State* state,
                    unordered_map<string, ushort_t>& stringAtomMap) {

    /* Sending the state description to the planning server. */
    ostringstream oss;
    oss << "state:" << state;
    char buffer[BUFFER_SIZE];
    bzero(buffer, BUFFER_SIZE);
    sprintf(buffer, "%s", oss.str().c_str());
    cout << "SENDING: " << oss.str() << endl;
    int n = write(sockfd, buffer, strlen(buffer));
    if (n < 0) {
        cerr << "ERROR: couldn't write to socket." << endl;
        return "";
    }
    bzero(buffer, BUFFER_SIZE);
    n = read(sockfd, buffer, BUFFER_SIZE - 1);
    if (n < 0) {
        cerr << "ERROR: couldn't read from socket." << endl;
        return "";
    }
    return string(buffer);
}


/* Simulates receiving an action description from the client. */
void stopServer(int sockfd) {

    /* Sending the state description to the planning server. */
    ostringstream oss;
    oss << "stop:";
    char buffer[BUFFER_SIZE];
    bzero(buffer, BUFFER_SIZE);
    sprintf(buffer, "%s", oss.str().c_str());
    cout << "SENDING: " << oss.str() << endl;
    int n = write(sockfd, buffer, strlen(buffer));
    if (n < 0)
        cerr << "ERROR: couldn't write to socket." << endl;
}

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


void printTree(PolicyTree* tree) {
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
    outfile.open("/home/naman/TMP_Merged/graph.gv");
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
    outfile.close();
    cout << "Done";





}


int main(int argc, char **argv)
{
    mdplib_debug = true;
//    int n;
//    cin >> n;

    string file;
    string prob;
    problem_t *problem = NULL;
    pair<state_t *,Rational> *initial = NULL;
cout << "One" << endl;

    if (argc < 2) {
        cout << "Usage: testClient [file] [problem]\n";
        exit(0);
    }

    file = argv[1];
    prob = argv[2];


	int horizon = atoi(argv[3]);
    int port = atoi(argv[4]);


    if( !read_file( file.c_str() ) ) {
        cout <<
            "<main>: ERROR: couldn't read problem file `" << file << endl;
        exit(-1);
    }
cout << "One_part2" << endl;
    problem = (problem_t*) problem_t::find( prob.c_str() );
    if( !problem ) {
        cout << "<main>: ERROR: problem `" << prob <<
            "' is not defined in file '" << file << "'" << endl;
        exit(-1);
    }
cout << "Two" << endl;
    /* Initializing problem */
    MLProblem = new mlppddl::PPDDLProblem(problem);
    mlppddl::PPDDLHeuristic* heuristic =
        new mlppddl::PPDDLHeuristic(MLProblem, mlppddl::FF);
    MLProblem->setHeuristic(heuristic);

    mlcore::State* currentState = MLProblem->initialState();

    /* Setting up the socket to communicate with server. */
    int portno = port;
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        cerr << "ERROR: couldn't open socket." << endl;
        exit(-1);
    }
    struct hostent *server = gethostbyname("localhost");
    if (server == NULL) {
        cout << "No host: localhost." << endl;
        close(sockfd);
        exit(-1);
    }
cout << "three" << endl;
    struct sockaddr_in serv_addr;
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        cerr << "ERROR: couldn't connect." << endl;
        close(sockfd);
        exit(-1);
    }
cout << "built socket" << endl;
    /* Initializing a map from atom names to atom indices. */
    unordered_map<string, ushort_t> stringAtomMap;
    initStringAtomMap(problem, stringAtomMap);

    mlcore::Action* action = nullptr;

    string actionDescription = getActionFromServer(sockfd, currentState, stringAtomMap);

    for (mlcore::Action* a : MLProblem->actions()) {
        ostringstream oss;
        oss << a;
        if (oss.str() == actionDescription){
            action = a;
        }
    }

    PolicyTreeNode* temp = new PolicyTreeNode(action,currentState,1.0,1);
    PolicyTree* tree = new PolicyTree();
    tree->insertNode(temp, nullptr);

    float tempProb;
	int tempLevel;

    mlcore::State* tempState = nullptr;
    mlcore::Action* tempAction = nullptr;
    std::queue<PolicyTreeNode* > q;
    q.push(temp);

    while(!q.empty()) {
        temp = q.front();
        q.pop();
        tempAction = temp->action;
        tempState = temp->state;
        tempProb = temp->prob;
		tempLevel = temp->level;
        cout << "Number of Elements in Queue :"  << q.size() << endl;
		if (tempLevel <= horizon){
		    for(mlcore::Successor& su : MLProblem->transition(tempState,tempAction)){
			
		        actionDescription = getActionFromServer(sockfd,su.su_state,stringAtomMap);
		        PolicyTreeNode* temp2;
		        for (mlcore::Action* a : MLProblem->actions()) {
		            ostringstream oss;
		            oss << a;
		            if (oss.str() == actionDescription){
		                action = a;
		            }
		        }
		        if(actionDescription == "(done)") {
		            temp2 = new PolicyTreeNode(action,su.su_state,tempProb * su.su_prob,tempLevel++);
		            tree->insertNode(temp2,temp);
		            mlcore::State* nextState = randomSuccessor(MLProblem,su.su_state,action);
		            if (MLProblem->goal(nextState))
		                cout << endl << "*********GOAL REACHED**********" << endl;
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
    printTree(tree);




    stopServer(sockfd);
    close(sockfd);
    delete heuristic;
    return 0;
}


