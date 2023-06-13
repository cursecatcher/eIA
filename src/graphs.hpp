#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <cassert>

using namespace std; 




class Vertex {
    vector<int> neighbors; 
    

public:
    const int id; 
    const int x, y; 

    Vertex(int n, int x, int y) : id(n), x(x), y(y) {}

    inline void add_neighbor( const int v_id ) {
        assert( v_id != id );
        neighbors.push_back( v_id ); 
    }

    inline const vector<int>& get_neighbors() const {
        return neighbors;
    }

    friend std::ostream& operator<<(std::ostream& os, const Vertex& v) {
        os << "Vertex " << v.id << "( " << v.x << ", " << v.y << ")";
        return os; 
    }
    
    bool operator==(const Vertex& other) const {
        return this->id == other.id; 
    }
};



class GraphMap {
    vector<Vertex*> vertices; 
    vector<int> adj_matrix; 

    inline int get_linear_coordinates(int x, int y) const {
        return y * n_vertices + x;  
    }

public:
    const int n_vertices; 

    GraphMap( int n_vertices ) 
    : n_vertices( n_vertices ), 
      adj_matrix( vector<int>( n_vertices * n_vertices, 0 )) {
        vertices.resize( n_vertices );
    }

    ~GraphMap() {
        for ( auto it = vertices.begin(); it != vertices.end(); ++it)
            delete *it; 
    }

    inline void add_vertex( int n, int x, int y ) {
        vertices.at(n) = new Vertex(n, x, y ); 
    }

    inline const Vertex* get_vertex( int id ) const { return vertices.at(id); }


    inline void add_edge( int u, int v, int w ) {
        int c_1 = get_linear_coordinates(u, v), 
            c_2 = get_linear_coordinates(v, u); 

        adj_matrix.at( c_1 ) = adj_matrix.at( c_2 ) = w; 
        vertices.at(u)->add_neighbor(v); 
        vertices.at(v)->add_neighbor(u); 
        
    }


    inline int get(int x, int y) const {
        return adj_matrix.at( get_linear_coordinates( x, y ) );
    }


    void visualize() const {
        
        for (int y = 0; y < n_vertices; ++y) {
            for (int x = 0; x < n_vertices; ++x ) {    
                std::cout << get(x, y);
                std::cout << " "; 
            }
            std::cout << '\n'; 
        }
        std::cout << "\n";    


        std::cout << "\n\n";


        for (const Vertex* v: vertices ) {
            std::cout << "Vertex " << v->id << ": "; 
            for (int n: v->get_neighbors())
                std::cout << n << " ";
            std::cout << "\n";
            
        }
    }


};


class GraphMapBuilder {
public:
    GraphMapBuilder(const string& filename, GraphMap*&map ) { 
        ifstream f( filename ); 
        string line; 
        
        int n_vertices, n_edges; 
        f >> n_vertices; 
        map = new GraphMap( n_vertices );

        std::cout << "INIT GRAPH W/ " << n_vertices << " vertices\n"; 
        int id, x, y;
        

        for (int i = 0; i < n_vertices; ++i) {
            f >> id >> line >> x >> y; 
            map->add_vertex( id, x, y );
        }

        f >> n_edges;
        int u, v, w;  

        for (int i = 0; i < n_edges; ++i ) {
            f >> u >> v >> w; 
            map->add_edge( u, v, w );
        }
    }
};


class GraphNode {
    int f, h; 

    GraphNode *parent = nullptr; 


    void compute_f_cost( const Vertex& goal ) {
        this->h = abs( v.x - goal.x ) + abs( v.y - goal.y ); 
        this->f = this->h + this->g_cost; 
    }

public: 
    const Vertex& v;
    const int g_cost;

    GraphNode( const Vertex& start, const Vertex& goal ) : v(start), g_cost(0) {
        compute_f_cost( goal );
    }

    GraphNode( GraphNode*& curr, const Vertex*& neighbor, int g_cost, const Vertex& goal ) : 
        v( *neighbor ), 
        g_cost( curr->g_cost + g_cost ),
        parent( curr ) {

        compute_f_cost( goal ); 
    }

    const GraphNode* get_parent_node() const {
        return parent;  
    }


    // Overload the less-than operator to define the ordering
    bool operator<(const GraphNode& other) const {
        return this->f > other.f; 
    }

    friend std::ostream& operator<<(std::ostream& os, const GraphNode& v) {
        os << "GraphNode " << v.v << ": " << v.f << " = " << v.g_cost << " + " << v.h << "\n";
        return os; 
    }
};


class A_star_GraphSearch {
    const GraphMap& graph; 
    vector<GraphNode*> closedset, openset; 
    int n_open = 0; 

    const Vertex *goal = nullptr;

    
    inline bool has_been_visited( const Vertex& v ) const {
        return this->closedset.at( v.id ) != nullptr;
    }

    inline bool has_been_visited( const int id ) const {
        return this->closedset.at( id ) != nullptr; 
    }

    inline void set_visited( GraphNode*& n ) {
        this->closedset.at( n->v.id ) = n; 
    }


    inline void update_node( GraphNode*& n ) {
        GraphNode** p = &(openset.at( n->v.id ));

        if ( *p == nullptr ) {
            //add a new node 
            *p = n; 
            ++n_open; 
        }
        else if ( (*p)->g_cost > n->g_cost ) {
            //replace node
            delete *p;
            *p = n; 
        }
    }

    GraphNode* get_next() {
        int i = 0, best_i = -1;

        for ( auto it = openset.begin(); it != openset.end(); ++it, ++i) {
            if (*it) {
                switch (best_i) {
                    case -1:    best_i = i; break; 
                    default:
                        if (*it < openset.at(best_i)) {
                            best_i = i; 
                        }
                }
            }
        }

        --n_open; 
        GraphNode* best = openset.at(best_i);
        openset.at(best_i) = nullptr;

        return best;
    }


    void reconstruct_path( vector<int>& plan_vector ) const {

        GraphNode* final = closedset.at( goal->id );
        
        for ( const GraphNode* p = final; p; p = p->get_parent_node() ) {
            plan_vector.push_back( p->v.id );
        }

        reverse( plan_vector.begin(), plan_vector.end() );

    }

    void release_memory() {
        for (auto it = closedset.begin(); it != closedset.end(); ++it)
            delete *it; 

        for (auto it = openset.begin(); it != openset.end(); ++it)
            delete *it; 
    }


public: 
    A_star_GraphSearch( const GraphMap& map ) 
    :   graph(map), 
        closedset( vector<GraphNode*>( graph.n_vertices, nullptr )), 
        openset( vector<GraphNode*>( graph.n_vertices, nullptr )) { 
    }



    bool find_path( const Vertex& initial, const Vertex& goal, vector<int>& solution_vector ) {
        std::cout << "A* algorithm from " << initial << " to " << goal << "...\n"; 
        this->goal = &goal; 

        solution_vector.clear(); 

        GraphNode* starting_node = new GraphNode( initial, goal );   
        update_node( starting_node );

        while ( n_open ) {
            GraphNode* curr = get_next(); 

            if ( curr->v == goal ) {
                std::cout << "SIAMO ARRIVATI DAJE\n"; 
                std::cout << "SOlution: " << *curr << "\n"; 

                set_visited( curr );
                reconstruct_path( solution_vector ); 
                break;
            }

            if ( ! has_been_visited( curr->v ) ) {
                for ( const int neighbor: curr->v.get_neighbors() ) {

                    if ( !has_been_visited( neighbor ) ) {
                        const Vertex* v_neighbor = graph.get_vertex( neighbor ); 
                        int we = graph.get( curr->v.id, neighbor );

                        GraphNode* new_node = new GraphNode( 
                            curr, 
                            v_neighbor, 
                            we, 
                            goal ); 
                        this->update_node( new_node );

                    }
                }

                this->set_visited( curr );
            }

        }
        release_memory();
        return solution_vector.size() > 0;
    }
};

