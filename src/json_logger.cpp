#include "json_logger.h"
#include <iomanip>

JSONLogger::JSONLogger() : first_entry(true) {
}

void JSONLogger::log_expanding(int id, int parent_id, double cost, 
                              const std::vector<sPath>& paths, 
                              const Conflict& conflict) {
    std::stringstream ss;
    
    if (!first_entry) {
        ss << ",\n";
    } else {
        first_entry = false;
    }
    
    ss << "    {\n";
    ss << "      \"id\": " << id << ",\n";
    ss << "      \"type\": \"expanding\",\n";
    if (parent_id > 0) {
        ss << "      \"pId\": " << parent_id << ",\n";
    } else {
        ss << "      \"pId\": null,\n";
    }
    ss << "      \"cost\": " << std::fixed << std::setprecision(10) << cost << ",\n";
    
    // Individual path fields
    for (size_t i = 0; i < paths.size(); ++i) {
        ss << "      \"path" << i << "\": \"" << escape_json_string(path_to_string(paths[i])) << "\",\n";
    }
    
    // Conflict
    if (conflict.agent1 >= 0) {
        ss << "      \"collision\": \"" << escape_json_string(conflict_to_string(conflict)) << "\"\n";
    } else {
        ss << "      \"collision\": null\n";
    }
    
    ss << "    }";
    
    log_entries.push_back(ss.str());
}

void JSONLogger::log_generating(int id, int parent_id, double cost,
                               const sPath& old_path, 
                               const std::list<Constraint>& old_constraints,
                               const Constraint& new_constraint,
                               const sPath& new_path) {
    std::stringstream ss;
    
    if (!first_entry) {
        ss << ",\n";
    } else {
        first_entry = false;
    }
    
    ss << "    {\n";
    ss << "      \"id\": " << id << ",\n";
    ss << "      \"type\": \"generating\",\n";
    if (parent_id > 0) {
        ss << "      \"pId\": " << parent_id << ",\n";
    } else {
        ss << "      \"pId\": null,\n";
    }
    ss << "      \"cost\": " << std::fixed << std::setprecision(10) << cost << ",\n";
    ss << "      \"old_path\": \"" << escape_json_string(path_to_string(old_path)) << "\",\n";
    
    // Individual constraint fields
    int constraint_index = 0;
    for (const auto& constraint : old_constraints) {
        ss << "      \"old_constraint" << constraint_index << "\": \"" << escape_json_string(constraint_to_string(constraint)) << "\",\n";
        constraint_index++;
    }
    
    ss << "      \"new_constraint\": \"" << escape_json_string(constraint_to_string(new_constraint)) << "\",\n";
    ss << "      \"new_path\": \"" << escape_json_string(path_to_string(new_path)) << "\"\n";
    ss << "    }";
    
    log_entries.push_back(ss.str());
}

std::string JSONLogger::path_to_string(const sPath& path) {
    std::stringstream ss;
    
    for (size_t i = 0; i < path.nodes.size(); ++i) {
        if (i > 0) ss << "->";
        
        // Use node ID only
        ss << "(" << path.nodes[i].id << ", " 
           << std::fixed << std::setprecision(10) << path.nodes[i].g << ")";
    }
    
    return ss.str();
}

std::string JSONLogger::constraints_to_string(const std::list<Constraint>& constraints) {
    std::stringstream ss;
    bool first = true;
    
    for (const auto& constraint : constraints) {
        if (!first) ss << ", ";
        ss << constraint_to_string(constraint);
        first = false;
    }
    
    return ss.str();
}

std::string JSONLogger::constraint_to_string(const Constraint& constraint) {
    std::stringstream ss;
    
    ss << "<Agent_" << constraint.agent << ", " 
       << constraint.id1 << "->" << constraint.id2 << ", ["
       << std::fixed << std::setprecision(10) << constraint.t1 << ","
       << std::fixed << std::setprecision(10) << constraint.t2 << ")";
    
    if (constraint.positive) {
        ss << " +";
    }
    
    ss << ">";
    
    return ss.str();
}

std::string JSONLogger::conflict_to_string(const Conflict& conflict) {
    std::stringstream ss;
    
    ss << "<Agent_" << conflict.agent1 << ", (" 
       << conflict.move1.id1 << "," << std::fixed << std::setprecision(10) << conflict.move1.t1 
       << ")->(" << conflict.move1.id2 << "," << std::fixed << std::setprecision(10) << conflict.move1.t2 
       << ")> vs <Agent_" << conflict.agent2 << ", (" 
       << conflict.move2.id1 << "," << std::fixed << std::setprecision(10) << conflict.move2.t1 
       << ")->(" << conflict.move2.id2 << "," << std::fixed << std::setprecision(10) << conflict.move2.t2 
       << ")>";
    
    return ss.str();
}

std::string JSONLogger::escape_json_string(const std::string& str) {
    std::string result;
    result.reserve(str.length());
    
    for (char c : str) {
        switch (c) {
            case '"':  result += "\\\""; break;
            case '\\': result += "\\\\"; break;
            case '\b': result += "\\b"; break;
            case '\f': result += "\\f"; break;
            case '\n': result += "\\n"; break;
            case '\r': result += "\\r"; break;
            case '\t': result += "\\t"; break;
            default:   result += c; break;
        }
    }
    
    return result;
}

void JSONLogger::write_to_file(const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "[\n";
        for (const auto& entry : log_entries) {
            file << entry;
        }
        file << "\n]\n";
        file.close();
    }
}

void JSONLogger::write_to_cout() {
    std::cout << "[\n";
    for (const auto& entry : log_entries) {
        std::cout << entry;
    }
    std::cout << "\n]\n";
}

void JSONLogger::clear() {
    log_entries.clear();
    first_entry = true;
} 