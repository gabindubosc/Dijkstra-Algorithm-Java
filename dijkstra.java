/*
 * Este código implementa o algoritmo de Dijkstra para encontrar os caminhos mais curtos em um grafo ponderado não direcionado.
 */

package javaapplication2;

import java.util.*;

public class JavaApplication2 {
    
    // Classe para representar um nó no grafo
    static class Node implements Comparable<Node> {
        int vertex; // Identificador do vértice
        int cost;   // Custo até este vértice
        
        // Construtor da classe Node
        public Node(int vertex, int cost) {
            this.vertex = vertex;
            this.cost = cost;
        }
        
        // Método de comparação para a PriorityQueue
        @Override
        public int compareTo(Node other) {
            return Integer.compare(this.cost, other.cost);
        }
    }

    // Método para encontrar o caminho mais curto usando o algoritmo de Dijkstra
    public static void dijkstra(Map<Integer, List<Node>> graph, int source, int n, String[] vertexNames) {
        PriorityQueue<Node> pq = new PriorityQueue<>(); // Fila de prioridade para os vértices a serem processados
        int[] dist = new int[n]; // Array para armazenar as distâncias mínimas
        int[] prev = new int[n]; // Array para armazenar os predecessores no caminho mínimo
        
        Arrays.fill(dist, Integer.MAX_VALUE); // Inicializa todas as distâncias como infinito
        Arrays.fill(prev, -1); // Inicializa todos os predecessores como -1
        dist[source] = 0; // Distância do vértice de origem para ele mesmo é zero
        pq.add(new Node(source, 0)); // Adiciona o vértice de origem à fila de prioridade

        // Algoritmo principal de Dijkstra
        while (!pq.isEmpty()) {
            Node currentNode = pq.poll(); // Extrai o vértice com menor custo atual
            int u = currentNode.vertex;

            // Itera sobre os vértices adjacentes do vértice atual
            for (Node neighbor : graph.get(u)) {
                int v = neighbor.vertex;
                int weight = neighbor.cost;

                // Verifica se encontrou um caminho mais curto para o vértice adjacente
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight; // Atualiza a distância mínima para v
                    prev[v] = u; // Atualiza o predecessor de v
                    pq.add(new Node(v, dist[v])); // Adiciona v à fila de prioridade com sua nova distância mínima
                }
            }
        }
        
        // Imprime as distâncias e os caminhos mais curtos do vértice de origem para todos os outros vértices
        for (int i = 0; i < n; i++) {
            if (i != source) {
                System.out.print("Distância de " + vertexNames[source] + " para " + vertexNames[i] + ": " + dist[i] + " - Caminho: ");
                printPath(prev, i, vertexNames); // Imprime o caminho mínimo até o vértice i
                System.out.println();
            }
        }
    }

    // Método auxiliar para imprimir o caminho do vértice de origem até o vértice destino
    public static void printPath(int[] prev, int vertex, String[] vertexNames) {
        List<Integer> path = new ArrayList<>();
        
        // Constrói o caminho reversamente começando do vértice destino
        for (int at = vertex; at != -1; at = prev[at]) {
            path.add(at);
        }
        
        Collections.reverse(path); // Inverte a lista para obter o caminho correto
        
        // Imprime o caminho no formato "A -> 1 -> ..."
        for (int i = 0; i < path.size(); i++) {
            System.out.print(vertexNames[path.get(i)]);
            if (i < path.size() - 1) {
                System.out.print(" -> ");
            }
        }
    }
    
    // Método auxiliar para adicionar uma aresta no grafo
    public static void addEdge(Map<Integer, List<Node>> graph, int u, int v, int weight) {
        graph.get(u).add(new Node(v, weight)); // Adiciona v como vértice adjacente de u com o peso da aresta
        graph.get(v).add(new Node(u, weight)); // Grafo não direcionado, portanto adiciona u como vértice adjacente de v também
    }

    public static void main(String[] args) {
        
        // Definindo o grafo como um mapa de listas de adjacência
        Map<Integer, List<Node>> graph = new HashMap<>();
        int n = 7; // Número de vértices no grafo

        // Inicializando as listas de adjacência para cada vértice
        for (int i = 0; i < n; i++) {
            graph.put(i, new ArrayList<>());
        }

        // Adicionando as arestas do grafo com seus pesos
        addEdge(graph, 0, 1, 20); // A - 1
        addEdge(graph, 0, 4, 16); // A - 20
        addEdge(graph, 1, 3, 34); // 1 - 18
        addEdge(graph, 1, 2, 35); // 1 - 16
        addEdge(graph, 4, 2, 31); // 20 - 16
        addEdge(graph, 4, 5, 16); // 20 - 23
        addEdge(graph, 4, 3, 16); // 20 - 18
        addEdge(graph, 4, 6, 34); // 20 - 34
        addEdge(graph, 3, 2, 16); // 18 - 16
        addEdge(graph, 2, 5, 25); // 16 - 23
        addEdge(graph, 5, 6, 41); // 23 - 34

        // Mapeamento de índices para nomes dos vértices
        String[] vertexNames = {"A", "1", "16", "18", "20", "23", "34"};
        
        // Executando o algoritmo de Dijkstra para cada vértice como fonte
        for (int source = 0; source < n; source++) {
            System.out.println("------------------");
            System.out.println("Calculando a partir de " + vertexNames[source] + ":");
            dijkstra(graph, source, n, vertexNames); // Executa Dijkstra para o vértice source
        }
    }

}