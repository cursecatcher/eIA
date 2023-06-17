#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <chrono>

#define VERBOSE 1 
// #define GRAPH 0
// #ifdef GRAPH
    #include "graphs.hpp"
// #endif 


using namespace std; 




enum AgentAction { UP, DOWN, LEFT, RIGHT, NOP }; 
enum TypeOfCell { OK, OBSTACLE, OUT_OF_MAP };


std::ostream& operator<<(std::ostream& os, const AgentAction& a) {
    switch (a) {
        case UP:        os << "UP"; break; 
        case DOWN:      os << "DOWN"; break; 
        case LEFT:      os << "LEFT"; break; 
        case RIGHT:     os << "RIGHT"; break; 
        default:        break; 
    }

    return os; 
}



// /* Just an auxiliary function to see priority queue content */
// template<typename Q>
// void print_queue(std::string name, Q q)
// {
//     std::cout << "PRIORITY QUEUE CONTENT:  ";
//     // NB: q is passed by value because there is no way to traverse
//     // priority_queue's content without erasing the queue.
//     for (std::cout << name << ":\n"; !q.empty(); q.pop())
//         std::cout << q.top() << '\n';
//     std::cout << "END\n\n";
// }


class Position {
    // identify a position given a (x, y) pair, where x = column and y = row 

    int x, y; 

    inline Position apply_up()      const {  return Position( x, y - 1 );    }
    inline Position apply_down()    const {  return Position( x, y + 1 );    }
    inline Position apply_left()    const {  return Position( x - 1, y );    }
    inline Position apply_right()   const {  return Position( x + 1, y );    }
public:
    Position() :x(-1), y(-1) {}
    Position(const int x, const int y ) : x(x), y(y) {}
    Position(const Position& p) : Position(p.x, p.y) {}

    inline void set(int x, int y) {
        this->x = x;    this->y = y; 
    }

    inline int get_x() const {   return x; }
    inline int get_y() const {   return y; }

    inline Position apply_action(const AgentAction a) const {
        switch ( a ) {
            case UP:        return apply_up(); 
            case DOWN:      return apply_down(); 
            case LEFT:      return apply_left();
            case RIGHT:     return apply_right(); 
            default:        return *this;  
        }
    }

    inline bool operator==(const Position& pos) const {
        return this->x == pos.x && this->y == pos.y; 
    }

    friend std::ostream& operator<<(std::ostream& os, const Position& obj) {
        os << "Position(" << obj.x << ", " << obj.y << ")";
        return os;
    }
}; 


class AgentStep {
    Position pos; 
    AgentAction action; 

public:
    AgentStep( const Position& p, const AgentAction& a) : pos(p), action(a) {}

    inline const Position& get_position() const { return pos; }
    inline const AgentAction get_action() const { return action; }

    friend std::ostream& operator<<(std::ostream& os, const AgentStep& obj) {
        os << "AgentStep: ( " << obj.pos << " --  Action = " << obj.action << " )"; 
        return os;
    }
};


typedef vector<AgentStep> AgentPlan; 



class Node {
    AgentStep state_info; 
    int f, g, h; 

    Node *parent = nullptr; 

    void compute_hf_costs(const Position& goal) {
        const Position& pos = state_info.get_position();    
        this->h = abs( pos.get_x() - goal.get_x() ) + abs( pos.get_y() - goal.get_y() );
        this->f = this->g + this->h;  
    }

public: 
    Node( const Position& s, const Position& goal ) 
    : state_info( s, NOP ), g ( 0 ) { 

        compute_hf_costs( goal ); 
    }

    Node( const Node& curr, AgentAction action, const Position& goal ) 
    : state_info( curr.state_info.get_position().apply_action( action ), action  ),
      g( curr.g_cost() + 1 ) {

        compute_hf_costs( goal ); 
    }

    Node ( Node*& curr, AgentAction action, const Position& goal ) 
    : Node( *curr, action, goal ) {
        parent = curr; 
    } 


    inline const AgentStep& get_state()     const { return state_info; }
    inline const Position& get_pos()        const { return state_info.get_position(); }
    inline const AgentAction get_action()   const { return state_info.get_action(); }
    inline int f_cost() const  {   return f;   }
    inline int g_cost() const  {   return g;   }
    inline int h_cost() const  {   return h;   }
    

    // Overload the less-than operator to define the ordering
    bool operator<(const Node& other) const {
        return this->f > other.f; 
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& obj) {
        os << obj.state_info.get_position() << "  -- costs: ( " << obj.f << ", " << obj.g << ", " << obj.h << " )"; 
        return os;
    }

    inline Node* get_parent_node() const { return parent; }

}; 






class Map {
    vector<int> the_matrix; 
    Position initial, goal; 

public:
    const int nrows, ncols; 

    Map(int nrows, int ncols) : nrows( nrows ), ncols( ncols ) {
        the_matrix.resize( nrows * ncols );
    }

    inline void set(int x, int y, int value) {
        the_matrix.at( get_linear_coordinates(x, y) ) = value; 
    }

    inline int get(int x, int y) const {
        return the_matrix.at( get_linear_coordinates( x, y ) );
    }

    void visualize(const Position& initial, const Position& goal) const {
        
        std::cout << "\nSTART: " << initial << "\nGOAL: " << goal << "\n\n"; 

        for (int y = 0; y < nrows; ++y) {
            for (int x = 0; x < ncols; ++x ) {
                if ( initial == Position(x, y))     std::cout << "P";
                else if ( goal == Position(x, y))   std::cout << "G";
                else                                std::cout << get(x, y);
                std::cout << " "; 
            }
            std::cout << '\n'; 
        }
        std::cout << "\n";    
    }


    void visualize(const AgentPlan& mov_plan ) const {
        vector<int> copymap( the_matrix ); 
        int tmp;

        for ( const AgentStep& step: mov_plan )
            copymap.at( get_linear_coordinates( step.get_position() ) ) = 3;

        for (int y = 0; y < nrows; ++y) {
            for (int x = 0; x < ncols; ++x ) {
                switch ( tmp = copymap.at( get_linear_coordinates( x, y ) ) ) {
                    case 0:     std::cout << "\033[1;34m" << tmp << "\033[0m ";  break;
                    case 1:     std::cout << "\033[1;32m" << tmp << "\033[0m ";  break; 
                    default:    std::cout << "\033[1;31m" << tmp << "\033[0m "; 
                } 
            }
            std::cout << "\n"; 
        }
    }

    inline int get_linear_coordinates(int x, int y) const {
        return y * ncols + x;  
    }

    inline int get_linear_coordinates( const Position& p ) const {
        return get_linear_coordinates( p.get_x(), p.get_y() );  
    }

    

    TypeOfCell check_position( const Position& p ) const {
        int x = p.get_x(), y = p.get_y(); 
        
        //check boundaries 
        if ( x < 0 || x >= ncols || y < 0 || y >= nrows ) 
            return OUT_OF_MAP;

        return get(x, y) == 1 ? OK : OBSTACLE; 
    }
};

class MapBuilder {
    int interpret_cell( const char c ) {
        switch ( c ) {
            case '1':   
            case 'S':
            case 'G':
                        return 1; 
            default:    return 0;
        }
    }


public: 
    MapBuilder(const string& filename, Map*&map, Position& initial, Position& final) {
        ifstream f( filename ); 
        string line; 
        vector<string> lines; 
        
        int ncols = 0; 

        while (getline(f, line)) {
            lines.emplace_back( line );
            ncols = line.size() > ncols ? line.size() : ncols; 
        }

        map = new Map( lines.size(), ncols ); 
        bool start = false, goal = false; 
        int x, y = 0; 

        for ( const string& curr: lines ) {

            if ( !start && (x = curr.find('S')) != string::npos ) {
                start = true; 
                initial.set( x, y ); 
            }
            if ( !goal && (x = curr.find('G')) != string::npos ) {
                goal = true; 
                final.set( x, y ); 
            }

            for (int x = 0; x < ncols; ++x) {
                map->set(x, y, this->interpret_cell( curr.at(x) ) );
            }

            ++y;
        }

    }
};



class A_star_Search {
    const Map& map;                  // the agent world 
    
    vector<Node*> frontier;         //a vector containing pointers to unexplored nodes that have to be visited 
    int n_open = 0; 

    vector<Node*> closedset;         //a vector containing visited nodes

    Position goal; 
    const vector<AgentAction> actions = { UP, DOWN, LEFT, RIGHT };

    bool has_been_visited( const Position& p) const {
        return this->closedset.at( map.get_linear_coordinates( p ) ) != nullptr; 
    }

    void set_visited( Node*& n ) {
        this->closedset.at( map.get_linear_coordinates( n->get_pos() ) ) = n; 
        // std::cout << "Node " << *n << " marked as visited\n"; 
    }


    void update_node( Node*& n ) {
        int i = map.get_linear_coordinates( n->get_pos() );
        Node** p = &(frontier.at(i)); 
        if ( *p ) {

            if ( (*p)->g_cost() > n->g_cost() ) { 
                delete *p;
                *p = n;
            }  
        }
        else {
            *p = n;
            ++n_open;
        }
    }

    Node* get_next() {
        int i = 0, best_i = -1;

        for ( auto it = frontier.begin(); it != frontier.end(); ++it, ++i) {
            if (*it) {
                switch (best_i) {
                    case -1:    best_i = i; break; 
                    default:
                        if (*it < frontier.at(best_i)) {
                            best_i = i; 
                        }
                }
            }
        }

        --n_open; 
        Node* best = frontier.at(best_i);
        frontier.at(best_i) = nullptr;
        return best;
    }



    void reconstruct_path( AgentPlan& plan_vector  ) {
        for ( Node *p = closedset.at( map.get_linear_coordinates( goal ) ); p; p = p->get_parent_node() ) { 
            plan_vector.push_back( p->get_state() );
        }

        reverse( plan_vector.begin(), plan_vector.end() );
    }

public: 
    A_star_Search( const Map& map ) : 
        map( map ), 
        frontier( vector<Node*>( map.nrows * map.ncols, nullptr )),
        closedset( vector<Node*>( map.nrows * map.ncols, nullptr )  ) {

        
    }

    bool find_path( const Position& initial, const Position& goal, AgentPlan& plan_vector ) { 
        this->goal = goal; 

        Node* starting_node = new Node( initial, goal ); 
        this->update_node( starting_node ); 
        

        while ( n_open ) {
            Node *curr = get_next();

            if ( curr->get_pos() == this->goal ) {
                // std::cout << "SIAMO ARRIVATI A " << *curr << "\nCHECK:" << endl; 
                this->set_visited( curr ); 
                this->reconstruct_path( plan_vector );

                return true; 
            }

            if ( ! this->has_been_visited( curr->get_pos() ) ) {
                // std::cout << "Expanding node " << *curr << endl; 

                for ( const AgentAction& a: actions ) {
                    
                    // double check: is the action applicable? 
                    // if so, check if the resulting state has already been visited
                    const Position future_pos = curr->get_pos().apply_action( a );
                    
                    if ( map.check_position( future_pos ) == OK && ! this->has_been_visited( future_pos ) ) {
                        // apply the action and build a new state 
                        Node* new_node = new Node( curr, a, goal );
                        // openset.push( new_node );
                        this->update_node( new_node );

                        #ifdef VERBOSE
                        std::cout << "New position available: " << *new_node << "\n";
                        #endif
                    }
                }

                this->set_visited( curr );        // marked current node as visited 
            }
        }

        return false; 
    }


};


int main( int argc, char **argv ) {
    if ( string( argv[1] ) == "map" ) {
        Map *map = nullptr; 
        Position initial, final; 
        

        MapBuilder( argv[2], map, initial, final );


        std::cout << "Initial: " << initial << "\nFinal: " << final << endl; 
        map->visualize(initial, final);


        auto start_t = chrono::_V2::steady_clock::now(); 
        AgentPlan plan; 
        A_star_Search searcher( *map ); 
        searcher.find_path( initial, final, plan ); 

        auto end_t = chrono::_V2::steady_clock::now(); 

        for (auto it = plan.begin(); it != plan.end(); ++it ) 
            std::cout << *it << "\n"; 

        // map->visualize(initial, final);

        map->visualize( plan ); 

        auto ns = chrono::duration_cast<chrono::nanoseconds>(end_t - start_t).count();
        auto ms = ns / 1000000;
        std::cout << "Planning time: " << ms  << "ms" << endl; 

        delete map; 
    }
    else {
        GraphMap *map = nullptr; 
        GraphMapBuilder( argv[2], map);
        
        int initial = std::stoi(argv[3]), 
            final = std::stoi( argv[4] ); 



        vector<int> plan_nodes; 

        auto start_t = chrono::_V2::steady_clock::now(); 

        A_star_GraphSearch a_star( *map ); 
        a_star.find_path( 
            *map->get_vertex(initial), *map->get_vertex(final), 
            plan_nodes );

        auto end_t = chrono::_V2::steady_clock::now(); 

        for ( const int v: plan_nodes ){
            std::cout << "Visit " << *(map->get_vertex( v ))     << endl; 
        }

        auto ns = chrono::duration_cast<chrono::nanoseconds>(end_t - start_t).count();
        auto mus = ns / 1000;
        std::cout << "Planning time: " << mus  << "microsec" << endl; 

        delete map;     
    }
}

