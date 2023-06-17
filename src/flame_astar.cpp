#include <iostream>
// #include <vector>
#include <array>
#include <chrono>
#include <tuple>
#include "graphs.hpp"

#define MANHATTAN_DISTANCE(x1,x2,y1,y2) (abs(x1-x2)+abs(y1-y2))
#define LINEAR_COORDINATES(x,y,N) (y * N + x)
#define NOT_PRESENT -1
#define STARTING_POINT -2 

//INDEXES FOR A_STAR_NODES tuples
#define F_COST 0 
#define G_COST 1
#define PREV   2

#define NUM_VERTICES 519

using namespace std; 

/* first: f(n)
 * second: g(n)
 * third: index of parent node   */
typedef tuple<int, int, int> a_star_opennode;




void a_star_flame( 
    const vector<int>& x_coords, 
    const vector<int>& y_coords, 
    const vector<int>& adj_matrix, 
    int start, int goal
);


void a_star_flame_v2(
    const size_t n_vertices, 
    const int* x_coords, 
    const int* y_coords, 
    const int* adj_matrix, 
    int start, int goal
); 




int main(int argc, char** argv) {
    GraphMap *map = nullptr; 
    GraphMapBuilder( argv[1], map);
    
    int initial = std::stoi(argv[2]), 
        final = std::stoi( argv[3] ); 


    vector<int> x_coords, y_coords; 

    for ( int i = 0; i < map->n_vertices; ++i) {
        const Vertex* v = map->get_vertex( i ); 
        x_coords.push_back( v->x );
        y_coords.push_back( v->y ); 

    }

    vector<int> adj_matrix( map->get_adj_matrix( ) );
    const int n_vertices = x_coords.size(); 



    a_star_flame( x_coords, y_coords, adj_matrix, initial, final );


    a_star_flame_v2( map->n_vertices, x_coords.data(), y_coords.data(), adj_matrix.data(), initial, final ); 


    delete map; 
}



void a_star_flame_v2(
    const size_t n_vertices, 
    const int* x_coords, 
    const int* y_coords, 
    const int* adj_matrix, 
    int start, int goal ) {


    int closedset[ n_vertices ] = { NOT_PRESENT };

    for (int i = 0; i < n_vertices; ++i) {
        std::cout << closedset[i] << " "; 
    }


    

}



void a_star_flame( 
    const vector<int>& x_coords, 
    const vector<int>& y_coords, 
    const vector<int>& adj_matrix, 
    int start, int goal ) {

    const int n_vertices = x_coords.size(); 
    // i-th vertex is closed (aka visited) if closedset[i] > 0
    vector<int> closedset( n_vertices, NOT_PRESENT );
    // i-th vertex is open (to be visited) is described by a pair (g, f)
    vector< a_star_opennode > openset( n_vertices, a_star_opennode(NOT_PRESENT, NOT_PRESENT, NOT_PRESENT) );
    int n_open = 0; 

    //initialize starting node 
    int initial_h = MANHATTAN_DISTANCE( x_coords[start], x_coords[goal], y_coords[start], y_coords[goal]);
    openset[start] = a_star_opennode( initial_h, 0, STARTING_POINT ); 
    ++n_open; 


    while( n_open ) {
        //1. identify next node to be expanded!
        int next_vertex = NOT_PRESENT; 

        for (int i = 0; i < n_vertices; ++i ) {
            if ( std::get<F_COST>( openset.at(i) ) != NOT_PRESENT ) {
                switch (next_vertex) {
                    case NOT_PRESENT:   
                        next_vertex = i;  
                        break; 
                    default:
                        if ( std::get<F_COST>( openset.at(i) ) < std::get<F_COST>( openset.at( next_vertex )) ) {
                            next_vertex = i; 
                        }
                        break;
                }
            }
        }

        //pick the selected node and remove it from the openset 
        a_star_opennode curr_node = openset.at( next_vertex );
        openset.at( next_vertex ) = a_star_opennode(NOT_PRESENT, NOT_PRESENT, NOT_PRESENT);
        --n_open; 

        // std::cout << "Current node: " << next_vertex << " target: " << goal << endl; 


        //2. CHECK IF IT MATCHES WITH THE GOAL : TODO 
        if ( next_vertex == goal ) {

            //HERE WE ARE 
            closedset.at( next_vertex ) = std::get<PREV>(curr_node);  
            
            std::cout << "START: " << start << ": (" << x_coords.at(start) << ", " << y_coords.at(start) << ")\n";
            std::cout << "GOAL: " << goal << ": (" << x_coords.at(goal) << ", " << y_coords.at(goal) << ")\n";
            

            int backtrack = closedset.at( next_vertex ); 
            while (backtrack != STARTING_POINT) {
                std::cout << "Visit " << backtrack << ": (" << x_coords.at(backtrack) << ", " << y_coords.at(backtrack) << ")\n"; 
                backtrack = closedset.at( backtrack );
            }


            return; 
        }

        //3. CHECK IF IT IS ALREADY VISITED (IS IT IN CLOSEDSET?)
        if (closedset.at( next_vertex ) == NOT_PRESENT ) {
            int curr_x = x_coords[ next_vertex ], curr_y = y_coords[ next_vertex ];
            //4. ADD EACH UNVISITED NEIGHBOR OF THE CURRENT NODE TO THE OPENSET 
            for (int i = 0; i < n_vertices; ++i) {
                int we = adj_matrix.at( LINEAR_COORDINATES(next_vertex, i, n_vertices) );
                // FOREACH UNVISITED NEIGHBOR...
                if ( we > 0 && closedset.at( i ) == NOT_PRESENT ) {
                    int new_g = std::get<G_COST>( curr_node ) + we;
                    int g_cost_neighbor = std::get<G_COST>( openset.at(i) );

                    //IF IS THE FIRST TIME THAT THE CURRENT NEIGHBOR HAS BEEN OPENED...
                    if ( g_cost_neighbor == NOT_PRESENT ) {
                        int h_cost = MANHATTAN_DISTANCE( x_coords[goal], x_coords[i], y_coords[goal], y_coords[i] );
                        openset.at( i ) = a_star_opennode( new_g + h_cost, new_g, next_vertex); // ADD NODE TO OPENSET !!!
                        ++n_open;
                    }
                    //IF THE NEW G COST IS LESS THAN THE CURRENT STORED, REPLACE IT 
                    else if ( new_g < g_cost_neighbor ) {
                        int h_cost = MANHATTAN_DISTANCE( x_coords[goal], x_coords[i], y_coords[goal], y_coords[i] );
                        openset.at( i ) = a_star_opennode( new_g + h_cost, new_g, next_vertex); //REPLACE NODE 
                    }
                }
            }

            closedset.at( next_vertex ) = std::get<PREV>(curr_node); 
        }
    }



}


// a_star_node build_node(
//     const vector<int>& x_coords, 
//     const vector<int>& y_coords, 
//     const vector<int>& adj_matrix, 
//     int current, int goal ) {

//     int h = abs( x_coords[current] - x_coords[goal] ) + abs( y_coords[current] - y_coords[goal]);
    
    

// }