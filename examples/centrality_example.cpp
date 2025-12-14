#include "graphix/vertex/algorithms/centrality.hpp"
#include "graphix/vertex/graph.hpp"
#include <iomanip>
#include <iostream>

using namespace graphix::vertex;
using namespace graphix::vertex::algorithms;

void print_header(const std::string &title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}

void print_centrality_map(const std::unordered_map<size_t, double> &centrality, const std::string &measure_name) {
    std::cout << "\n" << measure_name << ":\n";
    std::cout << std::string(40, '-') << "\n";

    // Get sorted entries by centrality value (descending)
    std::vector<std::pair<size_t, double>> sorted_entries(centrality.begin(), centrality.end());
    std::sort(sorted_entries.begin(), sorted_entries.end(),
              [](const auto &a, const auto &b) { return a.second > b.second; });

    for (const auto &[vertex, value] : sorted_entries) {
        std::cout << "Vertex " << vertex << ": " << std::fixed << std::setprecision(4) << value << "\n";
    }
}

// Example 1: Social Network - Find Influencers
void social_network_example() {
    print_header("Example 1: Social Network Analysis - Finding Influencers");

    Graph<void> network;

    // Create a small social network
    // Alice is connected to everyone (hub)
    auto alice = network.add_vertex();   // 0
    auto bob = network.add_vertex();     // 1
    auto charlie = network.add_vertex(); // 2
    auto diana = network.add_vertex();   // 3
    auto eve = network.add_vertex();     // 4
    auto frank = network.add_vertex();   // 5

    // Alice is the central connector
    network.add_edge(alice, bob, 1.0);
    network.add_edge(alice, charlie, 1.0);
    network.add_edge(alice, diana, 1.0);
    network.add_edge(alice, eve, 1.0);
    network.add_edge(alice, frank, 1.0);

    // Some other connections
    network.add_edge(bob, charlie, 1.0);
    network.add_edge(diana, eve, 1.0);
    network.add_edge(eve, frank, 1.0);

    std::cout << "Social network with " << network.vertex_count() << " people and " << network.edge_count()
              << " friendships.\n";

    // Compute all centrality measures
    auto degree_cent = degree_centrality_all(network);
    auto closeness_cent = closeness_centrality_all(network);
    auto betweenness_cent = betweenness_centrality_normalized(network);

    print_centrality_map(degree_cent, "Degree Centrality (Popularity)");
    print_centrality_map(closeness_cent, "Closeness Centrality (Accessibility)");
    print_centrality_map(betweenness_cent, "Betweenness Centrality (Bridge Role)");

    // Find the most influential person
    auto most_connected = most_central_vertex(degree_cent);
    auto most_accessible = most_central_vertex(closeness_cent);
    auto key_bridge = most_central_vertex(betweenness_cent);

    std::cout << "\nAnalysis Results:\n";
    std::cout << "- Most connected person (degree): Vertex " << most_connected << " (Alice)\n";
    std::cout << "- Most accessible person (closeness): Vertex " << most_accessible << " (Alice)\n";
    std::cout << "- Key bridge person (betweenness): Vertex " << key_bridge << " (Alice)\n";
    std::cout << "\nConclusion: Alice is the most influential person in this network!\n";
}

// Example 2: Transportation Network - Finding Critical Hubs
void transportation_network_example() {
    print_header("Example 2: Transportation Network - Finding Critical Hubs");

    Graph<void> transit;

    // Create a simplified subway/bus network
    auto downtown = transit.add_vertex();      // 0
    auto north_station = transit.add_vertex(); // 1
    auto south_station = transit.add_vertex(); // 2
    auto east_terminal = transit.add_vertex(); // 3
    auto west_terminal = transit.add_vertex(); // 4
    auto airport = transit.add_vertex();       // 5
    auto university = transit.add_vertex();    // 6

    // Downtown is the main hub
    transit.add_edge(downtown, north_station, 1.0);
    transit.add_edge(downtown, south_station, 1.0);
    transit.add_edge(downtown, east_terminal, 1.0);
    transit.add_edge(downtown, west_terminal, 1.0);

    // Other connections
    transit.add_edge(north_station, airport, 1.0);
    transit.add_edge(east_terminal, university, 1.0);
    transit.add_edge(south_station, university, 1.0);

    std::cout << "Transportation network with " << transit.vertex_count() << " stations.\n";

    auto betweenness = betweenness_centrality_normalized(transit);
    auto closeness = closeness_centrality_all(transit);

    print_centrality_map(betweenness, "Betweenness Centrality (Hub Importance)");
    print_centrality_map(closeness, "Closeness Centrality (Accessibility)");

    auto critical_hub = most_central_vertex(betweenness);

    std::cout << "\nCritical Infrastructure Analysis:\n";
    std::cout << "- Most critical hub (betweenness): Vertex " << critical_hub << " (Downtown)\n";
    std::cout << "- This station is critical for network connectivity\n";
    std::cout << "- Failure of this hub would severely impact the network!\n";
}

// Example 3: Citation Network (Directed Graph)
void citation_network_example() {
    print_header("Example 3: Academic Citation Network (Directed)");

    Graph<void> citations;

    // Create papers
    auto seminal_paper = citations.add_vertex(); // 0 - Old influential paper
    auto paper_a = citations.add_vertex();       // 1
    auto paper_b = citations.add_vertex();       // 2
    auto paper_c = citations.add_vertex();       // 3
    auto recent_paper = citations.add_vertex();  // 4
    auto review_paper = citations.add_vertex();  // 5

    // Citation links (directed: A cites B means A -> B)
    // Seminal paper is cited by everyone
    citations.add_edge(paper_a, seminal_paper, 1.0, EdgeType::Directed);
    citations.add_edge(paper_b, seminal_paper, 1.0, EdgeType::Directed);
    citations.add_edge(paper_c, seminal_paper, 1.0, EdgeType::Directed);
    citations.add_edge(review_paper, seminal_paper, 1.0, EdgeType::Directed);

    // Paper A is also influential
    citations.add_edge(paper_b, paper_a, 1.0, EdgeType::Directed);
    citations.add_edge(paper_c, paper_a, 1.0, EdgeType::Directed);

    // Recent work builds on earlier work
    citations.add_edge(recent_paper, paper_b, 1.0, EdgeType::Directed);
    citations.add_edge(recent_paper, paper_c, 1.0, EdgeType::Directed);
    citations.add_edge(recent_paper, paper_a, 1.0, EdgeType::Directed);

    // Review paper cites everything
    citations.add_edge(review_paper, paper_a, 1.0, EdgeType::Directed);
    citations.add_edge(review_paper, paper_b, 1.0, EdgeType::Directed);
    citations.add_edge(review_paper, paper_c, 1.0, EdgeType::Directed);
    citations.add_edge(review_paper, recent_paper, 1.0, EdgeType::Directed);

    std::cout << "Citation network with " << citations.vertex_count() << " papers.\n";

    auto degree_cent = degree_centrality_all(citations);
    auto betweenness = betweenness_centrality_normalized(citations);

    print_centrality_map(degree_cent, "Degree Centrality (Citation Count)");
    print_centrality_map(betweenness, "Betweenness Centrality (Knowledge Bridge)");

    std::cout << "\nAcademic Impact Analysis:\n";
    std::cout << "- Vertex 0 (Seminal Paper): Highly cited, foundational work\n";
    std::cout << "- Vertex 1 (Paper A): Also influential, cited by later work\n";
    std::cout << "- Vertex 5 (Review Paper): High out-degree, synthesizes knowledge\n";
}

// Example 4: Computer Network - Finding Bottlenecks
void computer_network_example() {
    print_header("Example 4: Computer Network - Finding Bottlenecks");

    Graph<void> network;

    // Create network topology
    auto gateway = network.add_vertex();      // 0 - Internet gateway
    auto router1 = network.add_vertex();      // 1
    auto router2 = network.add_vertex();      // 2
    auto server1 = network.add_vertex();      // 3
    auto server2 = network.add_vertex();      // 4
    auto workstation1 = network.add_vertex(); // 5
    auto workstation2 = network.add_vertex(); // 6
    auto workstation3 = network.add_vertex(); // 7

    // Gateway connects to routers
    network.add_edge(gateway, router1, 1.0);
    network.add_edge(gateway, router2, 1.0);

    // Routers connect to servers
    network.add_edge(router1, server1, 1.0);
    network.add_edge(router2, server2, 1.0);

    // Workstations connect to routers
    network.add_edge(router1, workstation1, 1.0);
    network.add_edge(router1, workstation2, 1.0);
    network.add_edge(router2, workstation3, 1.0);

    // Cross-connection for redundancy
    network.add_edge(router1, router2, 1.0);

    std::cout << "Computer network with " << network.vertex_count() << " nodes.\n";

    auto betweenness = betweenness_centrality_normalized(network);
    auto closeness = closeness_centrality_all(network);

    print_centrality_map(betweenness, "Betweenness Centrality (Bottleneck Risk)");

    // Find top 3 critical nodes
    auto top3 = top_k_central_vertices(betweenness, 3);

    std::cout << "\nNetwork Reliability Analysis:\n";
    std::cout << "Top 3 critical nodes (potential bottlenecks):\n";
    for (size_t i = 0; i < top3.size(); i++) {
        auto vertex_id = top3[i];
        auto centrality_value = betweenness.at(vertex_id);
        std::cout << "  " << (i + 1) << ". Vertex " << vertex_id << " (centrality: " << std::fixed
                  << std::setprecision(4) << centrality_value << ")\n";
    }
    std::cout << "\nRecommendation: Add redundancy around these critical nodes!\n";
}

// Example 5: Organizational Hierarchy
void organizational_hierarchy_example() {
    print_header("Example 5: Organizational Structure Analysis");

    Graph<void> org;

    // Create organizational structure (directed)
    auto ceo = org.add_vertex();             // 0
    auto cto = org.add_vertex();             // 1
    auto cfo = org.add_vertex();             // 2
    auto eng_manager = org.add_vertex();     // 3
    auto product_manager = org.add_vertex(); // 4
    auto engineer1 = org.add_vertex();       // 5
    auto engineer2 = org.add_vertex();       // 6
    auto engineer3 = org.add_vertex();       // 7

    // Reporting structure (reports to = directed edge)
    org.add_edge(cto, ceo, 1.0, EdgeType::Directed);
    org.add_edge(cfo, ceo, 1.0, EdgeType::Directed);
    org.add_edge(eng_manager, cto, 1.0, EdgeType::Directed);
    org.add_edge(product_manager, cto, 1.0, EdgeType::Directed);
    org.add_edge(engineer1, eng_manager, 1.0, EdgeType::Directed);
    org.add_edge(engineer2, eng_manager, 1.0, EdgeType::Directed);
    org.add_edge(engineer3, eng_manager, 1.0, EdgeType::Directed);

    // Cross-functional collaboration
    org.add_edge(engineer1, product_manager, 1.0, EdgeType::Directed);
    org.add_edge(engineer2, product_manager, 1.0, EdgeType::Directed);

    std::cout << "Organizational chart with " << org.vertex_count() << " positions.\n";

    auto degree_cent = degree_centrality_all(org);
    auto betweenness = betweenness_centrality_normalized(org);

    print_centrality_map(degree_cent, "Degree Centrality (Connections)");
    print_centrality_map(betweenness, "Betweenness Centrality (Communication Flow)");

    std::cout << "\nOrganizational Insights:\n";
    std::cout << "- CEO (Vertex 0): Low degree but structurally important\n";
    std::cout << "- Engineering Manager (Vertex 3): High betweenness, key coordinator\n";
    std::cout << "- CTO (Vertex 1): Hub for technical communication\n";
}

// Example 6: Disease Spread Network
void disease_spread_example() {
    print_header("Example 6: Disease Spread Modeling");

    Graph<void> contacts;

    // Create contact network
    auto patient_zero = contacts.add_vertex(); // 0
    auto person1 = contacts.add_vertex();      // 1
    auto person2 = contacts.add_vertex();      // 2
    auto person3 = contacts.add_vertex();      // 3
    auto person4 = contacts.add_vertex();      // 4
    auto person5 = contacts.add_vertex();      // 5
    auto person6 = contacts.add_vertex();      // 6
    auto person7 = contacts.add_vertex();      // 7

    // Contact links (undirected)
    contacts.add_edge(patient_zero, person1, 1.0);
    contacts.add_edge(patient_zero, person2, 1.0);
    contacts.add_edge(patient_zero, person3, 1.0);

    contacts.add_edge(person1, person4, 1.0);
    contacts.add_edge(person2, person5, 1.0);
    contacts.add_edge(person3, person6, 1.0);
    contacts.add_edge(person3, person7, 1.0);

    // Some interconnections
    contacts.add_edge(person4, person5, 1.0);
    contacts.add_edge(person6, person7, 1.0);

    std::cout << "Contact tracing network with " << contacts.vertex_count() << " individuals.\n";

    auto closeness = closeness_centrality_all(contacts);
    auto betweenness = betweenness_centrality_normalized(contacts);

    print_centrality_map(closeness, "Closeness Centrality (Infection Spread Speed)");
    print_centrality_map(betweenness, "Betweenness Centrality (Super-spreader Potential)");

    auto super_spreader = most_central_vertex(betweenness);

    std::cout << "\nEpidemiological Analysis:\n";
    std::cout << "- Patient Zero (Vertex 0): High centrality, infection source\n";
    std::cout << "- Super-spreader potential: Vertex " << super_spreader << "\n";
    std::cout << "- Priority for intervention: High centrality individuals\n";
}

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║        CENTRALITY MEASURES - REAL-WORLD EXAMPLES          ║\n";
    std::cout << "║                    Graphix Library                         ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";

    social_network_example();
    transportation_network_example();
    citation_network_example();
    computer_network_example();
    organizational_hierarchy_example();
    disease_spread_example();

    print_header("Summary");
    std::cout << "\nCentrality measures help identify important vertices:\n";
    std::cout << "  • Degree Centrality: Number of connections (popularity)\n";
    std::cout << "  • Closeness Centrality: Average distance to others (accessibility)\n";
    std::cout << "  • Betweenness Centrality: Bridge between communities (control)\n";
    std::cout << "\nApplications demonstrated:\n";
    std::cout << "  1. Social networks (influencers)\n";
    std::cout << "  2. Transportation networks (critical hubs)\n";
    std::cout << "  3. Citation networks (influential papers)\n";
    std::cout << "  4. Computer networks (bottlenecks)\n";
    std::cout << "  5. Organizations (key coordinators)\n";
    std::cout << "  6. Disease spread (super-spreaders)\n";
    std::cout << "\n";

    return 0;
}
