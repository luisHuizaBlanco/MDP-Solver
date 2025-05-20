#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <regex>
#include <cmath>
#include <iomanip>

class Node
{
public:
    std::string name;
    int reward = 0;

    //booleans for the node's type
    bool isDecisionNode = false;
    bool isTerminal = false;
    bool hasOneEdge = false;
    double decisionProb = 1.0;

    //<name of node, probability>
    std::unordered_map <std::string, double> transitions;

    Node()
    {
        name = "";
        reward = 0;
        isDecisionNode = false;
        isTerminal = false;
        hasOneEdge = false;
        decisionProb = 1.0;
    }
    
    Node(const std::string& nme, int rwrd)
    {
        name = nme;
        reward = rwrd;
    }

    //adds edges to the node
    void addTransition(const std::string& dest, double prob)
    {
        transitions[dest] = prob;
    }

    //used to label characteristics of the node important for the MDP
    void setTags(bool isDec = false, bool isTer = false, bool oneEdge = false)
    {
        isDecisionNode = isDec;
        isTerminal = isTer;
        hasOneEdge = oneEdge;
    }

    //sets the probability used for decision nodes
    void setDecProb(double prob)
    {
        decisionProb = prob;
    }
};

class MDP
{

public:

    //functions to populate the MDP with nodes and the informations for said nodes
    void addNode(const std::string& nme, int rwrd)
    {
        nodes[nme] = Node(nme, rwrd);
    }

    void addTransition(const std::string& nme, const std::string& dest, double prob)
    {
        if(nodes.find(nme) == nodes.end()) 
        {
            addNode(nme, 0);
        }

        nodes[nme].addTransition(dest, prob);
    }

    void setNodeTags(const std::string& nme, bool isDec = false, bool isTer = false, bool oneEdge = false)
    {
        nodes[nme].setTags(isDec, isTer, oneEdge);
    }

    void setDecisionProbability(const std::string& nme, double decProb)
    {
        nodes[nme].setDecProb(decProb);
    }

    std::unordered_map<std::string, Node> nodes;

    std::unordered_map<std::string, double> V;

    std::unordered_map<std::string, std::string> policy;
};

void printMDP(MDP& m)
{
    //details all the information from the nodes of the MDP given in the file

    for (const auto& [name, node] : m.nodes) 
    {
        // Print node type
        std::cout << "Node " << name << ":\n";
        if(node.isTerminal) 
        {
            std::cout << "  Type: Terminal\n";
        } 
        else if(node.isDecisionNode) 
        {
            std::cout << "  Type: Decision\n";
            std::cout << "  Decision Probability: " << node.decisionProb << "\n";
        }
        else if(node.hasOneEdge)
        {
            std::cout << "  Type: Has One Edge\n";
        } 
        else 
        {
            std::cout << "  Type: Chance\n";
        }

        // Print reward
        std::cout << "  Reward: " << node.reward << "\n";

        // Print transitions
        if(node.transitions.empty()) 
        {
            std::cout << "  No transitions (Terminal Node).\n";
        } 
        else 
        {
            std::cout << "  Transitions:\n";
            for (const auto& [dest, prob] : node.transitions) 
            {
                std::cout << "    -> " << dest;

                // Add probability details for chance nodes
                if(!node.isTerminal && !node.isDecisionNode) 
                {
                    std::cout << " with probability " << prob;
                }

                std::cout << "\n";
            }
        }

        // Indicate if it has exactly one edge
        if(node.hasOneEdge) 
        {
            std::cout << "  This node has exactly one transition.\n";
        }

        std::cout << std::endl; // Separate nodes with an empty line for readability
    }
}

MDP buildMDP(std::string _file)
{
    MDP mdp;

    std::ifstream file(_file);
    std::string line;

    //opening the files with the data for the mdp, if the file isnt found the program stops
    if(!file.is_open())
    {
        std::cout << "Error: file could not be opened" << std::endl;
        exit(-1);
    }

    //setting up storage for the data of mdp
    std::vector<std::string> names;
    std::unordered_map<std::string, std::string> edges; 
    std::unordered_map<std::string, double> probabilities;
    std::unordered_map<std::string, int> rewards;

    std::regex reward_regex(R"((\w+)\s*=\s*(-?\d+))");
    std::regex edge_regex(R"((\w+)\s*:\s*\[([^\]]+)\])");
    std::regex probability_regex(R"((\w+)\s*%\s*([0-9.\s]+))");

    //parsing the files data using regex
    while (std::getline(file, line)) 
    {
        std::smatch match;

        //this is used to skip any comment or empty lines in the file
        line = line.substr(0, line.find('#'));
        if(line.empty()) continue;

        if(std::regex_match(line, match, reward_regex)) 
        {   
            //assign rewards
            std::string name = match[1];
            int reward = std::stoi(match[2]);
            rewards[name] = reward;
            names.push_back(name);
        } 
        else if(std::regex_match(line, match, edge_regex)) 
        {
            //assign edges
            std::string name = match[1];
            edges[name] = match[2];
            names.push_back(name);
        } 
        else if(std::regex_match(line, match, probability_regex)) 
        {
            //probabilities of transitions
            std::string name = match[1];
            std::istringstream prob_stream(match[2]);
            float prob;
            std::vector<float> prob_list;

            while (prob_stream >> prob) 
            {
                prob_list.push_back(prob);
            }

            if(prob_list.size() == 1) 
            {
                probabilities[name] = prob_list[0];
            } 
            else 
            {
                for (size_t i = 0; i < prob_list.size(); ++i) 
                {
                    probabilities[name + std::to_string(i)] = prob_list[i];
                }
            }
            names.push_back(name);
        } 
        else 
        {
            //if an incorrect format is found, the program ends
            std::cout << "Unknown line format: " << line << std::endl;
            exit(1);
        }
    }

    file.close();

    //clear the names vector of any repeat entries done during parsing
    std::sort(names.begin(), names.end());
    names.erase(std::unique(names.begin(), names.end()), names.end());

    //begin populating the MDP with the information of the nodes
    for (const auto& name : names) 
    {
        int reward = rewards.count(name) ? rewards[name] : 0;

        //begin setup with the name and reward of the node
        mdp.addNode(name, reward);

        //get all the edges of the current node
        std::vector<std::string> edge_list;
        if(edges.count(name)) 
        {
            std::istringstream edge_stream(edges[name]);
            std::string edge;

            // Collect edges for the current node
            while (std::getline(edge_stream, edge, ',')) 
            {
                edge.erase(std::remove_if(edge.begin(), edge.end(), ::isspace), edge.end());
                edge_list.push_back(edge);
            }
        }

        //get the probabilities of the edges
        std::vector<double> node_probabilities;
        for (size_t i = 0; i < edge_list.size(); ++i) 
        {
            if(probabilities.count(name + std::to_string(i))) 
            {
                node_probabilities.push_back(probabilities[name + std::to_string(i)]);
            }
        }

        if(probabilities.count(name)) 
        {
            node_probabilities.push_back(probabilities[name]);
        }

        //we get the number of edges and probabilities to determine if
        //node is terminal, has only one edge, or if its a decision or chance node
        size_t num_probabilities = node_probabilities.size();
        size_t num_edges = edge_list.size();

        bool isDecisionNode = false;
        bool isTerminal = (num_edges == 0);   
        bool hasOneEdge = (num_edges == 1);
        double decisionProbability = 1.0;

        if(num_probabilities == 1 && num_edges > 1) 
        {
            isDecisionNode = true;
            decisionProbability = node_probabilities[0];
        } 
        else if(num_probabilities == 0 && num_edges > 1) 
        {
            isDecisionNode = true;
            decisionProbability = 1.0;
        }

        std::unordered_map<std::string, double> edge_probabilities;
        for (size_t i = 0; i < edge_list.size(); ++i) 
        {
            if(isDecisionNode) 
            {
                edge_probabilities[edge_list[i]] = 1.0; // Decision nodes: all edges get probability 1
            } 
            else if(probabilities.count(name + std::to_string(i))) 
            {
                edge_probabilities[edge_list[i]] = probabilities[name + std::to_string(i)];
            } 
            else if(probabilities.count(name)) 
            {
                edge_probabilities[edge_list[i]] = probabilities[name];
            } 
            else 
            {
                edge_probabilities[edge_list[i]] = 1.0; // Default to probability 1.0
            }
        }

        //populate the node with its edges
        for (const auto& edge : edge_list) 
        {
            mdp.addTransition(name, edge, edge_probabilities[edge]);
        }

        //setup the tags for the node
        mdp.setNodeTags(name, isDecisionNode, isTerminal, hasOneEdge);

        //set decision probability if the node is a decision node
        if(isDecisionNode) 
        {
            mdp.setDecisionProbability(name, decisionProbability);
        }
    }

    // checks if there are any nodes referenced in the edges that have no instance of being declared
    for (const auto& [name, node] : mdp.nodes) 
    {
        for (const auto& [dest, _] : node.transitions) 
        {
            if(mdp.nodes.find(dest) == mdp.nodes.end()) 
            {
                std::cout << "Error: Edge from node " << name << " points to non-existent node " << dest << ".\n";
                exit(1);
            }
        }
    }

    return mdp;
}

std::unordered_map<std::string, double> valueIteration(MDP& mdp, float disc_factor, double tol, int value_iteration, bool max, bool verbose)
{
    std::unordered_map<std::string, double> V;

    //initialize V
    for (const auto& [name, node] : mdp.nodes)
    {
        V[name] = node.reward;
    }

    //begin iteration
    for (int iter = 0; iter < value_iteration; iter++)
    {
        if(verbose) std::cout << "iteration: " << iter << std::endl;

        bool has_converged = true;
        std::unordered_map<std::string, double> new_V = V;

        for (const auto& [name, node] : mdp.nodes) 
        {
            if(verbose) std::cout << "iterating on node: " << name << std::endl;

            //skipping terminal nodes
            if(node.isTerminal) 
            {
                new_V[name] = V[name];
                if(verbose) std::cout << "Terminal node " << name << " has reward: " << new_V[name] << std::endl;
                continue;
            }

            double future_reward = 0.0;

            if(node.isDecisionNode) 
            {
                if(verbose) std::cout << "Decision Node " << name << std::endl;
                // Decision node: Maximize over possible actions

                double best_value = max ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();

                size_t num_transitions = node.transitions.size();

                for (const auto& [dest, prob] : node.transitions) 
                {
                    double action_value = V[dest] * node.decisionProb;

                    if(verbose) std::cout << "Going to node " << dest << " gives reward = " << action_value << std::endl;

                    double other_rewards = 0.0;

                    for (const auto& [other_dest, _] : node.transitions) 
                    {
                        if(other_dest != dest) // Exclude the primary transition
                        {
                            other_rewards += V[other_dest];
                        }
                    }

                    double other_probability = 1 - node.decisionProb;

                    if(verbose) std::cout << "Others Prob = " << other_probability << std::endl;

                    other_probability = other_probability / (num_transitions - 1);

                    other_rewards = other_rewards * other_probability;

                    if(verbose) std::cout << "Others Reward = " << other_rewards << std::endl;

                    action_value = action_value + other_rewards;

                    best_value = max ? std::max(best_value, action_value) : std::min(best_value, action_value);
                    
                }

                future_reward = best_value;

                if(verbose) std::cout << "Future Reward = " << future_reward << std::endl;

                new_V[name] = node.reward + disc_factor * future_reward;
            }
            else if(node.hasOneEdge)
            {
                //node has one edge, only one way for it to go
                const auto& [dest,_] = *node.transitions.begin();

                future_reward = V[dest];

                if(verbose) std::cout << "Future Reward = " << future_reward << std::endl;

                new_V[name] = node.reward + future_reward * disc_factor;
            } 
            else 
            {
                if(verbose) std::cout << "Chance Node " << name << std::endl;
                // Chance node: Use weighted sum of transition probabilities
                for (const auto& [dest, prob] : node.transitions) 
                {
                    future_reward += prob * V[dest];
                }

                new_V[name] = node.reward + future_reward * disc_factor;

                if(verbose) std::cout << "Future Reward = " << future_reward << std::endl;
            }

            if(verbose) std::cout << "node " << name << " reward = " << new_V[name] << std::endl;

            // Check if the value has converged
            if(std::abs(new_V[name] - V[name]) > tol) 
            {
                if(verbose) std::cout << "variance from previous iteration: " << std::abs(new_V[name] - V[name]) << std::endl;
                has_converged = false;
            }
            else
            {
                if(verbose) std::cout << "value has converged: " << new_V[name] << std::endl;
            }
        }

        // Update V with the new values
        V = new_V;

        // Stop if values have converged
        if(has_converged) 
        {
            break;
        }
    }
    
    return V;
}

std::unordered_map<std::string, std::string> Greedy(MDP& mdp, std::unordered_map<std::string, double>& V, bool max)
{
    std::unordered_map<std::string, std::string> new_policy;

    for (const auto& [name, node] : mdp.nodes) 
    {
        if(!node.isDecisionNode) continue;

        // Find the action that maximizes value
        std::string best_action;

        double best_value = max ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();

        for (const auto& [dest, prob] : node.transitions) 
        {
            double action_value = prob * V.at(dest);

            if((max && action_value > best_value) || (!max && action_value < best_value)) 
            {
                best_value = action_value;
                
                best_action = dest;
            }
        }
        
        new_policy[name] = best_action;
    }

    return new_policy;
}

void SolveMDP(MDP& mdp, float disc_factor, double tol, int value_iteration, bool max, bool verbose)
{
    std::unordered_map<std::string, double> V;
    std::unordered_map<std::string, std::string> policy;

    std::unordered_map<std::string, std::string> new_policy;

    for (const auto& [name, node] : mdp.nodes) 
    {
        if(node.isDecisionNode) 
        {
            policy[name] = node.transitions.begin()->first;
        }
    }

    //loop

    while(true)
    {        
        V = valueIteration(mdp, disc_factor, tol, value_iteration, max, verbose);

        new_policy = Greedy(mdp, V, max);

        if(new_policy == policy)
        {
            break;
        }
        else
        {
            policy = new_policy;
        }
    }

    mdp.V = V;

    mdp.policy = policy;
}

void printSolution(MDP& mdp) 
{
    std::map<std::string, std::string> sorted_policy(mdp.policy.begin(), mdp.policy.end());

    for (const auto& [state, action] : sorted_policy) 
    {
        std::cout <<  state << " -> " << action << "\n";
    }

    std::map<std::string, double> sorted_V(mdp.V.begin(), mdp.V.end());
        
    std::cout << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& [state, value] : sorted_V) 
    {
        std::cout << state << "=" << value << " ";
    }
}

int main(int argc, char** argv)
{
    std::string arg_string;
    float discount_factor = 1.0;
    bool maximize = false;
    double tolerance = 0.001;
    int value_iteration = 150;
    std::string file_name;
    bool verbose = false;

    if(argc < 2)
    {
        std::cout << "Error: no arguments given" << std::endl;
        return 1;
    }

    //turn argv into strings, then parse them as needed

    for(int i = 1; i < argc; i++)
    {
        arg_string += argv[i];
        arg_string += " ";
    }

    std::istringstream iss(arg_string);
    std::string arg;

    bool df = false;
    bool tol = false;
    bool iter = false;

    while(iss >> arg)
    {  
        if(arg == "-df")
        {
            df = true;
            continue;
        }

        if(df)
        {
            discount_factor = std::stof(arg);
            df = false;
            continue;
        }

        if(arg == "-max")
        {
            maximize = true;
            continue;
        }

        if(arg == "-v")
        {
            verbose = true;
            continue;
        }

        if(arg == "-tol")
        {
            tol = true;
            continue;
        }

        if(tol)
        {
            tolerance = std::stod(arg);
            tol = false;
            continue;
        }

        if(arg == "-iter")
        {
            iter = true;
            continue;
        }

        if(iter)
        {
            value_iteration = std::stoi(arg);
            iter = false;
            continue;
        }

        file_name = arg;
    }

    MDP mdp = buildMDP(file_name);

    if(verbose) std::cout << "disc factor = " << discount_factor << " tolerance = " << tolerance << " value iter = " << value_iteration << " max set to " << (maximize ? "true" : "false") << std::endl;

    if(verbose) printMDP(mdp);
    
    SolveMDP(mdp, discount_factor, tolerance, value_iteration, maximize, verbose);

    printSolution(mdp);

}