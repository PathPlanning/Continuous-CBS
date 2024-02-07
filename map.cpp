#include "map.h"

bool Map::get_map(const char* FileName)
{
    tinyxml2::XMLElement *root = nullptr;
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error! Cannot open Map XML file!" << std::endl;
        return false;
    }
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (root)
    {
        return get_grid(FileName);
    }
    else
    {
        std::cout << "Only grid-maps are supproted in AA-CCBS!" << std::endl;
        return false;
    }
}

int Map::get_id(int i, int j) const
{
    return i*width+j;
}

double Map::get_i(int id) const
{
    return int(id/width);
}

double Map::get_j(int id) const
{
    return int(id%width);
}

bool Map::get_grid(const char* FileName)
{

    tinyxml2::XMLElement *root = nullptr, *map = nullptr, *element = nullptr, *mapnode = nullptr;

    std::string value;
    std::stringstream stream;
    bool hasGridMem(false), hasGrid(false), hasHeight(false), hasWidth(false);

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }
    map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "Error! No '" << CNS_TAG_MAP << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (mapnode = map->FirstChildElement(); mapnode; mapnode = mapnode->NextSiblingElement())
    {
        element = mapnode->ToElement();
        value = mapnode->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        stream.str("");
        stream.clear();
        stream << element->GetText();

        if (!hasGridMem && hasHeight && hasWidth)
        {
            grid.resize(height);
            for (int i = 0; i < height; ++i)
                grid[i].resize(width);
            hasGridMem = true;
        }

        if (value == CNS_TAG_HEIGHT)
        {
            if (hasHeight)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_HEIGHT << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_HEIGHT << "' =" << height << "will be used."
                          << std::endl;
            }
            else
            {
                if (!((stream >> height) && (height > 0)))
                {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_HEIGHT
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_HEIGHT << "' tag should be an integer >=0" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_HEIGHT
                              << "' tag will be encountered later..." << std::endl;
                }
                else
                    hasHeight = true;
            }
        }
        else if (value == CNS_TAG_WIDTH)
        {
            if (hasWidth)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_WIDTH << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_WIDTH << "' =" << width << "will be used." << std::endl;
            }
            else
            {
                if (!((stream >> width) && (width > 0)))
                {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_WIDTH
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_WIDTH << "' tag should be an integer AND >0" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_WIDTH
                              << "' tag will be encountered later..." << std::endl;

                }
                else
                    hasWidth = true;
            }
        }
        else if (value == CNS_TAG_GRID)
        {
            int grid_i(0), grid_j(0);
            hasGrid = true;
            if (!(hasHeight && hasWidth))
            {
                std::cout << "Error! No '" << CNS_TAG_WIDTH << "' tag or '" << CNS_TAG_HEIGHT << "' tag before '"
                          << CNS_TAG_GRID << "'tag encountered!" << std::endl;
                return false;
            }
            element = mapnode->FirstChildElement();
            while (grid_i < height)
            {
                if (!element)
                {
                    std::cout << "Error! Not enough '" << CNS_TAG_ROW << "' tags inside '" << CNS_TAG_GRID << "' tag."
                              << std::endl;
                    std::cout << "Number of '" << CNS_TAG_ROW
                              << "' tags should be equal (or greater) than the value of '" << CNS_TAG_HEIGHT
                              << "' tag which is " << height << std::endl;
                    return false;
                }
                std::string str = element->GetText();
                std::vector<std::string> elems;
                std::stringstream ss(str);
                std::string item;
                while (std::getline(ss, item, ' '))
                    elems.push_back(item);
                grid_j = 0;
                int val;
                if (elems.size() > 0)
                    for (grid_j = 0; grid_j < width; ++grid_j)
                    {
                        if (grid_j == int(elems.size()))
                            break;
                        stream.str("");
                        stream.clear();
                        stream << elems[grid_j];
                        stream >> val;
                        grid[grid_i][grid_j] = val;
                    }

                if (grid_j != width)
                {
                    std::cout << "Invalid value on " << CNS_TAG_GRID << " in the " << grid_i + 1 << " " << CNS_TAG_ROW
                              << std::endl;
                    return false;
                }
                ++grid_i;
                element = element->NextSiblingElement();
            }
        }
    }
    if (!hasGrid) {
        std::cout << "Error! There is no tag 'grid' in xml-file!\n";
        return false;
    }
    size = width*height;
    return true;
}

void Map::print_map()
{
    std::cout<<height<<"x"<<width<<std::endl;
    for(int i = 0; i < height; i++)
    {
        std::cout<<"<row>";
        for(int j = 0; j < width; j++)
            std::cout<<grid[i][j]<<" ";
        std::cout<<"</row>"<<std::endl;
    }
}

void Map::printPPM()
{
    std::cout<<"P3\n"<<width<<" "<<height<<"\n255\n";
    for(int i = 0; i < height; i++)
        for(int j = 0; j < width; j++)
        {
            if(grid[i][j]==1)
                std::cout<<"0 0 0\n";
            else
                std::cout<<"255 255 255\n";
        }
}

bool Map::cell_on_grid(int i, int j) const
{
    return i >= 0 && j >= 0 && i < height and j < width;
}

bool Map::cell_is_obstacle(int i, int j) const
{
    return (grid[i][j] == CN_OBSTL);
}

bool Map::check_line(int x1, int y1, int x2, int y2)
{
    int delta_x(std::abs(x1 - x2));
    int delta_y(std::abs(y1 - y2));
    if((delta_x > delta_y && x1 > x2) || (delta_y >= delta_x && y1 > y2))
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    int step_x(x1 < x2 ? 1 : -1);
    int step_y(y1 < y2 ? 1 : -1);
    int error(0), x(x1), y(y1);
    int gap = int(agent_size*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON);
    int k, num;

    if(delta_x > delta_y)
    {
        int extraCheck = int(agent_size*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON);
        for(int n = 1; n <= extraCheck; n++)
        {
            error += delta_y;
            num = (gap - error)/delta_x;
            for(k = 1; k <= num; k++)
                if(cell_is_obstacle(x1 - n*step_x, y1 + k*step_y))
                    return false;
            for(k = 1; k <= num; k++)
                if(cell_is_obstacle(x2 + n*step_x, y2 - k*step_y))
                    return false;
        }
        error = 0;
        for(x = x1; x != x2 + step_x; x++)
        {
            if(cell_is_obstacle(x, y))
                return false;
            if(x < x2 - extraCheck)
            {
                num = (gap + error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(cell_is_obstacle(x, y + k*step_y))
                        return false;
            }
            if(x > x1 + extraCheck)
            {
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(cell_is_obstacle(x, y - k*step_y))
                        return false;
            }
            error += delta_y;
            if((error<<1) > delta_x)
            {
                y += step_y;
                error -= delta_x;
            }
        }
    }
    else
    {
        int extraCheck = int(agent_size*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON);
        for(int n = 1; n <= extraCheck; n++)
        {
            error += delta_x;
            num = (gap - error)/delta_y;
            for(k = 1; k <= num; k++)
                if(cell_is_obstacle(x1 + k*step_x, y1 - n*step_y))
                    return false;
            for(k = 1; k <= num; k++)
                if(cell_is_obstacle(x2 - k*step_x, y2 + n*step_y))
                    return false;
        }
        error = 0;
        for(y = y1; y != y2 + step_y; y += step_y)
        {
            if(cell_is_obstacle(x, y))
                return false;
            if(y < y2 - extraCheck)
            {
                num = (gap + error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(cell_is_obstacle(x + k*step_x, y))
                        return false;
            }
            if(y > y1 + extraCheck)
            {
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(cell_is_obstacle(x - k*step_x, y))
                        return false;
            }
            error += delta_x;
            if((error<<1) > delta_y)
            {
                x += step_x;
                error -= delta_y;
            }
        }
    }
    return true;
}
