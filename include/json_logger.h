#ifndef JSON_LOGGER_H
#define JSON_LOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include "structs.h"

class JSONLogger {
private:
    std::vector<std::string> log_entries;
    bool first_entry;
    
public:
    JSONLogger();
    
    // Log node expansion
    void log_expanding(int id, int parent_id, double cost, 
                      const std::vector<sPath>& paths, 
                      const Conflict& conflict);
    
    // Log node generation (when creating child nodes)
    void log_generating(int id, int parent_id, double cost,
                       const sPath& old_path, 
                       const std::list<Constraint>& old_constraints,
                       const Constraint& new_constraint,
                       const sPath& new_path);
    
    // Helper functions
    std::string path_to_string(const sPath& path);
    std::string constraints_to_string(const std::list<Constraint>& constraints);
    std::string constraint_to_string(const Constraint& constraint);
    std::string conflict_to_string(const Conflict& conflict);
    std::string escape_json_string(const std::string& str);
    
    // Output functions
    void write_to_file(const std::string& filename);
    void write_to_cout();
    void clear();
};

#endif // JSON_LOGGER_H 