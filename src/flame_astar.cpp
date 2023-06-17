#include <iostream>
// #include <vector>
#include <array>
#include <chrono>
#include <tuple>
#include <cstring>
#include "graphs.hpp"

// SOME MACROS SINCE WE CANNOT DEFINE FUNCTIONS 
#define MANHATTAN_DISTANCE(x1,x2,y1,y2) (abs(x1-x2)+abs(y1-y2))
#define LINEAR_COORDINATES(x,y,N) (y * N + x)

// SPECIAL VALUES FOR SPECIAL MEANINGS
#define NOT_PRESENT -1
#define STARTING_POINT -2 



//THE TOTAL NUMBER OF VERTICES IN THE GRAPH MAP (we need a constant value to use std::array)
#define NV 519
//THE MAXIMUM LENGTH OF A* SOLUTION 
#define SOL_LENGTH 10

using namespace std; 

/*  A* algorithm to find the shortest path from START to GOAL.
    It works on a undirected weighted graph composed of NV vertices, which are places identified by a set of coordinates c=(x, y).
    Edges connecting paires of vertices (v1,v2) are weighted w.r.t. the Manhattan distance between c(v1) and c(v2).

    A* gets as input the graph, namely the set of x and y coordinates of the vertices, and the adjacency matrix representing the edges. 
    A* explores the search space using the objective function 
        f(n)=g(n)+h(n)
    where g(n) is the ACTUAL cost to travel from START to n, and h(n) is the heuristic cost to travel from n to GOAL (Manhattan distance).
    Nodes are described by a triple (F_COST, G_COST, PREV) where PREV is the previous visited vertex.
*/

//INDEXES FOR A_STAR_NODES tuples
#define F_COST 0 
#define G_COST 1
#define PREV   2
/*  A_STAR_NODE = (F_COST, G_COST, PREV index) */
typedef tuple<int, int, int> a_star_opennode;

/*
    The function f(n) has to be minimized. A* works using two arrays (openset and closedset) of length NV. 
    The openset stores triples (F_COST, G_COST, PREV), while the closedset stores PREV values for vertices marked as visited, 
    The algorithm starts from START. f(START) = h(START), since g(START) = 0. It is add to the open set.
    1. The node with minimum f is extracted from the open set and explored,
        1a. if the node is the goal, the end e tutti a casa.
    2. Add all its unvisited neighbors to the openset. 
    3. For each vertex, the openset stores the solution such that f(v) is minimized. 
       Nodes in the openset can be updated if the new G_COST is less than the current one.
    4. The node is marked as visited, namely removed from the openset and the PREV vertex is stored in the closed. Go back to 1. 
 */




void a_star_flame_c_style(
    const int* x_coords, 
    const int* y_coords, 
    const int* adj_matrix, 
    int start, int goal, 
    int* solution ) {


    int closedset[ NV ];
    a_star_opennode openset[ NV ]; 

    for (int i = 0; i < NV; ++i) {
        closedset[ i ] = NOT_PRESENT; 
        openset[ i ] = a_star_opennode(NOT_PRESENT, NOT_PRESENT, NOT_PRESENT); 
    }

    //initialize starting node 
    int initial_h = MANHATTAN_DISTANCE( x_coords[start], x_coords[goal], y_coords[start], y_coords[goal]);
    openset[start] = a_star_opennode( initial_h, 0, STARTING_POINT ); 

    // Keep looping WHILE there are elements in the open set 
    for (int n_open = 1; n_open; ) {
        //1a. identify next node to be expanded!
        int next_vertex = NOT_PRESENT; 

        for (int i = 0, curr_f_cost; i < NV; ++i ) {
            if ( ( curr_f_cost = std::get<F_COST>( openset[i] ) ) != NOT_PRESENT ) {
                if ( next_vertex == NOT_PRESENT || curr_f_cost < std::get<F_COST>( openset[next_vertex]) ) {
                    next_vertex = i; 
                }
            }
        }

        //1b. pick the selected node and remove it from the openset 
        a_star_opennode curr_node = openset[next_vertex];
        openset[next_vertex] = a_star_opennode(NOT_PRESENT, NOT_PRESENT, NOT_PRESENT);
        --n_open; 


        //2. CHECK IF IT MATCHES WITH THE GOAL 
        if ( next_vertex == goal ) {
            int backward_solution[10], length = 0; 


            //HERE WE ARE 
            closedset[next_vertex] = std::get<PREV>(curr_node);  

            for ( int backtrack = closedset[next_vertex]; backtrack != STARTING_POINT; backtrack = closedset[backtrack] ) 
                backward_solution[ length++ ] = backtrack; 
            
            for (int i = 0, j = length - 1; i < length; ++i, --j)
                solution[i] = backward_solution[j];
            solution[length] = goal; 
            return; 
        }

        //3. CHECK IF IT IS ALREADY VISITED (IS IT IN CLOSEDSET?)
        if (closedset[next_vertex] == NOT_PRESENT ) {
            int curr_x = x_coords[ next_vertex ], curr_y = y_coords[ next_vertex ];
            //4. ADD EACH UNVISITED NEIGHBOR OF THE CURRENT NODE TO THE OPENSET 
            for (int i = 0; i < NV; ++i) {
                int we = adj_matrix[ LINEAR_COORDINATES(next_vertex, i, NV) ];
                // FOREACH UNVISITED NEIGHBOR...
                if ( we > 0 && closedset[i] == NOT_PRESENT ) {
                    int new_g = std::get<G_COST>( curr_node ) + we;
                    int g_cost_neighbor = std::get<G_COST>( openset[i] );
                    /* a new node is added to the open set if 
                       it not present yet in the openset, 
                       or if the new cost is less than the current present in the openset.  */
                    bool first_time = g_cost_neighbor == NOT_PRESENT, 
                         not_new_but_interesting = new_g < g_cost_neighbor;
                    
                    //if it is the first time, the node is added to the openset and the number of elements is ++increased 
                    if ( (first_time && ++n_open) || not_new_but_interesting ) {
                        // ADD NEW OR REPLACE NODE IN OPENSET estimating f(n)
                        openset[i] = a_star_opennode( 
                            new_g + MANHATTAN_DISTANCE( x_coords[i], x_coords[goal], y_coords[i], y_coords[goal] ),
                            new_g, 
                            next_vertex
                        ); 
                    }
                }
            }

            closedset[next_vertex] = std::get<PREV>(curr_node); 
        }
    }
}


void a_star_flame_array( 
    const array<int, NV>& x_coords, 
    const array<int, NV>& y_coords, 
    const array<int, NV*NV>& adj_matrix, 
    int start, int goal, 
    array<int, 10>& solution ) {

    
    array<int, NV> closedset;
    array< a_star_opennode, NV> openset;
    int n_open_nodes = 0; 

    closedset.fill( NOT_PRESENT ); 
    openset.fill( a_star_opennode( NOT_PRESENT, NOT_PRESENT, NOT_PRESENT ) ); 

    //initialize starting node 
    int initial_h = MANHATTAN_DISTANCE( x_coords[start], x_coords[goal], y_coords[start], y_coords[goal]);
    openset[start] = a_star_opennode( initial_h, 0, STARTING_POINT ); 

    // Keep looping WHILE there are elements in the open set 
    for (int n_open = 1; n_open; ) {
        //1a. identify next node to be expanded!
        int next_vertex = NOT_PRESENT; 

        for (int i = 0, curr_f_cost; i < NV; ++i ) {
            if ( ( curr_f_cost = std::get<F_COST>( openset.at(i) ) ) != NOT_PRESENT ) {
                if ( next_vertex == NOT_PRESENT || curr_f_cost < std::get<F_COST>( openset.at( next_vertex )) ) {
                    next_vertex = i; 
                }
            }
        }

        //1b. pick the selected node and remove it from the openset 
        a_star_opennode curr_node = openset.at( next_vertex );
        openset.at( next_vertex ) = a_star_opennode(NOT_PRESENT, NOT_PRESENT, NOT_PRESENT);
        --n_open; 


        //2. CHECK IF IT MATCHES WITH THE GOAL 
        if ( next_vertex == goal ) {
            int backward_solution[10], length = 0; 


            //HERE WE ARE 
            closedset.at( next_vertex ) = std::get<PREV>(curr_node);  
        
            for ( int backtrack = closedset.at( next_vertex ); backtrack != STARTING_POINT; backtrack = closedset.at( backtrack ) ) 
                backward_solution[ length++ ] = backtrack; 
            
            for (int i = 0, j = length - 1; i < length; ++i, --j)
                solution[i] = backward_solution[j];
            solution[length] = goal; 

            return; 
        }

        //3. CHECK IF IT IS ALREADY VISITED (IS IT IN CLOSEDSET?)
        if (closedset.at( next_vertex ) == NOT_PRESENT ) {
            int curr_x = x_coords[ next_vertex ], curr_y = y_coords[ next_vertex ];
            //4. ADD EACH UNVISITED NEIGHBOR OF THE CURRENT NODE TO THE OPENSET 
            for (int i = 0; i < NV; ++i) {
                int we = adj_matrix.at( LINEAR_COORDINATES(next_vertex, i, NV) );
                // FOREACH UNVISITED NEIGHBOR...
                if ( we > 0 && closedset.at( i ) == NOT_PRESENT ) {
                    int new_g = std::get<G_COST>( curr_node ) + we;
                    int g_cost_neighbor = std::get<G_COST>( openset.at(i) );
                    /* a new node is added to the open set if 
                       it not present yet in the openset, 
                       or if the new cost is less than the current present in the openset.  */
                    bool first_time = g_cost_neighbor == NOT_PRESENT, 
                         not_new_but_interesting = new_g < g_cost_neighbor;
                    
                    //if it is the first time, the node is added to the openset and the number of elements is ++increased 
                    if ( (first_time && ++n_open) || not_new_but_interesting ) {
                        // ADD NEW OR REPLACE NODE IN OPENSET estimating f(n)
                        openset.at( i ) = a_star_opennode( 
                            new_g + MANHATTAN_DISTANCE( x_coords[i], x_coords[goal], y_coords[i], y_coords[goal] ),
                            new_g, 
                            next_vertex
                        ); 
                    }
                }
            }

            closedset.at( next_vertex ) = std::get<PREV>(curr_node); 
        }
    }
}


void a_star_flame_vector( 
    const vector<int>& x_coords, 
    const vector<int>& y_coords, 
    const vector<int>& adj_matrix, 
    int start, int goal, 
    int* solution
    ) {

    const int n_vertices = x_coords.size(); 
    // i-th vertex is closed (aka visited) if closedset[i] > 0
    vector<int> closedset( n_vertices, NOT_PRESENT );
    // i-th vertex is open (waiting to be visited) 
    vector< a_star_opennode > openset( n_vertices, a_star_opennode(NOT_PRESENT, NOT_PRESENT, NOT_PRESENT) );

    //initialize starting node 
    int initial_h = MANHATTAN_DISTANCE( x_coords[start], x_coords[goal], y_coords[start], y_coords[goal]);
    openset[start] = a_star_opennode( initial_h, 0, STARTING_POINT ); 

    // Keep looping WHILE there are elements in the open set 
    for (int n_open = 1; n_open; ) {
        //1a. identify next node to be expanded!
        int next_vertex = NOT_PRESENT; 

        for (int i = 0, curr_f_cost; i < n_vertices; ++i ) {
            if ( ( curr_f_cost = std::get<F_COST>( openset.at(i) ) ) != NOT_PRESENT ) {
                if ( next_vertex == NOT_PRESENT || curr_f_cost < std::get<F_COST>( openset.at( next_vertex )) ) {
                    next_vertex = i; 
                }
            }
        }

        //1b. pick the selected node and remove it from the openset 
        a_star_opennode curr_node = openset.at( next_vertex );
        openset.at( next_vertex ) = a_star_opennode(NOT_PRESENT, NOT_PRESENT, NOT_PRESENT);
        --n_open; 


        //2. CHECK IF IT MATCHES WITH THE GOAL 
        if ( next_vertex == goal ) {
            int backward_solution[10], length = 0; 

            //HERE WE ARE 
            closedset.at( next_vertex ) = std::get<PREV>(curr_node);  
            
            for ( int backtrack = closedset.at( next_vertex ); backtrack != STARTING_POINT; backtrack = closedset.at( backtrack ) ) 
                backward_solution[ length++ ] = backtrack; 
            
            for (int i = 0, j = length - 1; i < length; ++i, --j)
                solution[i] = backward_solution[j];
            solution[length] = goal; 
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
                    /* a new node is added to the open set if 
                       it not present yet in the openset, 
                       or if the new cost is less than the current present in the openset.  */
                    bool first_time = g_cost_neighbor == NOT_PRESENT, 
                         not_new_but_interesting = new_g < g_cost_neighbor;
                    
                    //if it is the first time, the node is added to the openset and the number of elements is ++increased 
                    if ( (first_time && ++n_open) || not_new_but_interesting ) {
                        // ADD NEW OR REPLACE NODE IN OPENSET estimating f(n)
                        openset.at( i ) = a_star_opennode( 
                            new_g + MANHATTAN_DISTANCE( x_coords[i], x_coords[goal], y_coords[i], y_coords[goal] ),
                            new_g, 
                            next_vertex
                        ); 
                    }
                }
            }

            closedset.at( next_vertex ) = std::get<PREV>(curr_node); 
        }
    }
}



// void get_solution(int *solution)

void show_solution(
    const int* x_coords, 
    const int* y_coords, 
    const int* solution) {
    for (int i = 0; i < SOL_LENGTH && solution[i] != NOT_PRESENT; ++i)
        std::cout << "Vertex " << solution[i] << ": (" << x_coords[ solution[i] ] << ", " << y_coords[ solution[i] ] << ")\n";
    std::cout << "\n"; 
}



int main(int argc, char** argv) {
    /* LOAD THE MAP and BUILD THE GRAPH*/
    GraphMap *map = nullptr; 
    GraphMapBuilder( argv[1], map);
    
    int initial = std::stoi(argv[2]), 
        final = std::stoi( argv[3] ); 

    /* SINCE WE ARE IN 2023 WE CAN USE VECTORS  */
    vector<int> x_coords, y_coords; 

    for ( int i = 0; i < map->n_vertices; ++i) {
        const Vertex* v = map->get_vertex( i ); 
        x_coords.push_back( v->x );
        y_coords.push_back( v->y ); 

    }

    vector<int> adj_matrix( map->get_adj_matrix( ) );
    const int n_vertices = x_coords.size(); 

    int solution_a[SOL_LENGTH] = {NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT}, 
        solution_c[SOL_LENGTH] = {NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT, NOT_PRESENT};
    array<int, SOL_LENGTH> solution_b; 
    solution_b.fill(NOT_PRESENT);


    std::cout << "Using std::vector\n";
    a_star_flame_vector( x_coords, y_coords, adj_matrix, initial, final, solution_a );
    show_solution( x_coords.data(), y_coords.data(), solution_a );


    /* WE ARE IN 2023 BUT ACTUALLY WE CANNOT USE VECTORS */
    array<int, NV> ax_coords, ay_coords; 
    array<int, NV * NV> aadj_matrix; 

    for (int i = 0; i < NV; ++i) {
        ax_coords[i] = x_coords[i]; 
        ay_coords[i] = y_coords[i]; 

        for (int j = 0; j < NV; ++j) {
            int lin_coord = LINEAR_COORDINATES(i, j, NV); 
            aadj_matrix[ lin_coord ] = adj_matrix[ lin_coord ]; 
        }
    }

    std::cout << "Using std::array\n";
    a_star_flame_array( ax_coords, ay_coords, aadj_matrix, initial, final, solution_b );
    show_solution( ax_coords.data(), ay_coords.data(), solution_b.data() );


    /* WE STILL ARE IN 2023 BUT WE LIKE USE C-STYLE PROGRAMMING */
    std::cout << "Pretending to use C-style arrays\n";
    a_star_flame_c_style( ax_coords.data(), ay_coords.data(), aadj_matrix.data(), initial, final, solution_c ); 
    show_solution( ax_coords.data(), ay_coords.data(), solution_c );

    delete map; 
}


