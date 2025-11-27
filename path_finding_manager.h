//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <chrono>
#include <sstream>
#include <iostream>

using namespace std;


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    AStar,
    BestFirstSearch,
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    int render_counter = 0;

    struct Entry {
        Node* node;
        double dist;
        double priority; // Se usará para BestFirstSearch y A*

        bool operator < (const Entry& other) const {
            return priority > other.priority;
        }
    };

    double heuristica(Node* a, Node* b) const {
        double dx = a->coord.x - b->coord.x;
        double dy = a->coord.y - b->coord.y;
        return sqrt(dx * dx + dy * dy);
    }


    void dijkstra(Graph& graph) {
        unordered_map<Node*, double> dist;
        unordered_map<Node*, Node*> parent;
        unordered_set<Node*> closed;

        // Inicializamos distancias
        for (auto& p : graph.nodes)
            dist[p.second] = numeric_limits<double>::infinity();
        dist[src] = 0.0;

        priority_queue<Entry> pq;
        pq.push({src, 0.0, 0.0});

        while (!pq.empty()) {
            Entry top = pq.top();
            pq.pop();

            Node* u = top.node;

            // Si ya se visito, pasar
            if (closed.count(u))
                continue;
            closed.insert(u);

            // Si se llego al destino, corta
            if (u == dest)
                break;

            for (Edge* e : u->edges) {
                Node* v = nullptr;

                if (e->src == u)
                    v = e->dest;
                else if (!e->one_way && e->dest == u)
                    v = e->src;
                else
                    continue;

                if (closed.count(v))
                    continue;

                double new_cost = dist[u] + e->length;

                if (new_cost < dist[v]) {
                    dist[v] = new_cost;
                    parent[v] = u;

                    pq.push({v, new_cost, new_cost});

                    // Guardar arista visitada
                    visited_edges.emplace_back(
                        u->coord, v->coord,
                        sf::Color::Blue, 1.0f
                    );

                    // Renderizado por pasos
                    if (++render_counter % 40 == 0)
                        render();
                }
            }
        }

        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        // TODO: Add your code here

        set_final_path(parent);
    }

    void best_first_search(Graph &graph) {
        unordered_map<Node *, Node *> parent;
        unordered_set<Node *> closed;

        priority_queue<Entry> pq;
        pq.push({src, 0.0, heuristica(src, dest)}); // priority = solo h

        while (!pq.empty()) {
            auto top = pq.top();
            pq.pop();

            Node *u = top.node;

            if (closed.count(u))
                continue;
            closed.insert(u);

            // Si se llego al destino, corta
            if (u == dest)
                break;

            for (Edge *e: u->edges) {
                Node *v = nullptr;

                if (e->src == u)
                    v = e->dest;
                else if (!e->one_way && e->dest == u)
                    v = e->src;
                else
                    continue;

                if (closed.count(v))
                    continue;

                parent[v] = u;

                pq.push({v, 0.0, heuristica(v, dest)}); // solo heurística

                visited_edges.emplace_back(
                    u->coord, v->coord,
                    sf::Color::Blue, 1.0f
                );

                if (++render_counter % 40 == 0)
                    render();
            }
        }

        set_final_path(parent);
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        if (!current_graph) return;

        auto& win = window_manager->get_window();
        window_manager->clear();

        // Dibujar nodos
        for (const auto& pair : current_graph->nodes) {
            pair.second->draw(win);
        }

        // Dibujar aristas visitadas
        for (const auto& e : visited_edges) {
            e.draw(win, sf::RenderStates::Default);
        }

        // Dibujar inicio y destino
        if (src)  src->draw(win);
        if (dest) dest->draw(win);

        window_manager->display();
        sf::sleep(sf::milliseconds(1));
    }


    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(const std::unordered_map<Node*, Node*>& parent) {
        path.clear();

        Node* cursor = dest;

        while (cursor) {
            auto it = parent.find(cursor);
            if (it == parent.end()) break;

            Node* previous = it->second;
            if (previous) {
                path.emplace_back(previous->coord, cursor->coord,
                                  sf::Color::Red, 3.0f);
            }

            cursor = previous;
        }
    }

    Graph* current_graph = nullptr;

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph& g, Algorithm a) {
        if (!src || !dest)
            return;

        current_graph = &g;
        path.clear();
        visited_edges.clear();
        render_counter = 0;

        switch (a) {
            case Dijkstra:
                cout << "Usando algoritmo: Dijkstra\n";
                break;
            case AStar:
                cout << "Usando algoritmo: A*\n";
                break;
            case BestFirstSearch:
                cout << "Usando algoritmo: Best First Search\n";
                break;
            default:
                cout << "Funciones disponibles: D (Dijkstra), A (A*), B (Best-First), R (Reset), E (Extra), Q (Salir)\n";
                break;
        }
        auto begin = chrono::high_resolution_clock::now();

        if (a == Dijkstra)
            dijkstra(g);
        else if (a == AStar)
            a_star(g);
        else if (a == BestFirstSearch)
            best_first_search(g);

        auto finish = chrono::high_resolution_clock::now();
        auto ms = chrono::duration_cast<chrono::milliseconds>(finish - begin);

        cout << "Duracion: " << ms.count() << " ms\n";
        cout << "Visitados: " << visited_edges.size() << "\n";
        cout << "Segmentos de camino: " << path.size() << "\n";
        cout << "\n";
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
