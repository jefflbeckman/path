#include "Path.h"
#include <assert.h>
#include <algorithm> 
#include <fstream>
#include <iostream>
#include <string>
#include <memory>
using namespace Shapes;

#define PATH_VERSION_NUMBER 1

Path::~Path()
{
	//vectors should delete their contents
	//if not, then we can use std::auto_ptr for them
}
//Constructors
//rules: fixed_period must be set

Path::Path(size_t size)
{
	this->points.reserve(size);
	this->velocities.reserve(size);
	this->timestamps.reserve(size);
	this->fixed_period = 0;
}

Path::Path(size_t size, timestamp period)
{
	this->points.reserve(size);
	this->velocities.reserve(size);
	this->fixed_period = period;

}
Path::Path(std::vector<point> given_points, timestamp period)
{
	this->points = given_points;
	this->fixed_period = period;
	// do we calculate velocitys here?
}

Path::Path(char* filename)
{
	read_points_from_file(filename);
	this->fixed_period = 0;
}
Path::Path(std::vector<point> given_points, std::vector<timestamp> variable_periods)
{
	this->points = given_points;

	this->timestamps = variable_periods;
	this->fixed_period = 0;
}


Error Path::dump_to_file(char* filename)
{
	std::ofstream dump_file(filename);

	if (dump_file.fail())
	{
		return FILEIO;
	}
	//Header format subject to change with version, so always put version number field first
	dump_file << "Path Version: " << PATH_VERSION_NUMBER << "\n" << "length: " << points.size() << "\n";

	for (unsigned int i = 0; i < points.size(); i++){
		dump_file << points[i].x << " " << points[i].y << " " << points[i].z << " " << get_time(i) << "\n";
	}	

	return SUCCESS;
}
Error Path::read_points_from_file(char* filename)
{
	std::ifstream input_file(filename);

	if (input_file.fail())
	{
		return FILEIO;
	}

	// Read in the header
	std::string header_info;
	int version_number, length;

	input_file >> header_info;
	if (header_info.compare("Path") != 0)
	{
		return HEADER;
	}

	input_file >> header_info >> version_number;

	if (header_info.compare("Version:") != 0)
	{
		return HEADER;
	}

	switch (version_number)
	{
	case 1:
		input_file >> header_info >> length;
		if (header_info.compare("length:") != 0)
		{
			return HEADER;
		}  



		float x, y, z;
		timestamp time, last_time;

		if (fixed_period)
		{
			last_time = points.size() * fixed_period;
		}
		else
		{
			if (timestamps.empty())
			{
				last_time = 0;
			}
			else
			{
				last_time = timestamps.back();
			}
		}
		for (int i = 0; i < length; ++i)
		{
			input_file >> x >> y >> z >> time;
			add_point({ x, y, z }, last_time + time);
		}		
		break;

	default:
		return HEADER;
	}
	return SUCCESS;
}

Error Path::read_accel_from_file(char* filename)
{
	//TODO  
	return SUCCESS;
}


point Path::get_point(timestamp time)
{
	return this->points.at(get_index_from_time(time));
}

std::vector<point> Path::get_points_data()
{
	return this->points;
}

timestamp Path::get_fixed_period_data()
{
	return this->fixed_period;
}


std::vector<timestamp> Path::get_time_data()
{
	return this->timestamps;
}

Error Path::add_point(point added_point)
{
	assert(fixed_period!=0);
	points.push_back(added_point);
	vecolities_calculated = false;

	return SUCCESS;

}
Error Path::add_point(point added_point, timestamp time)
{
	// if this messes up the fixed period optimization, 
	// then populate the timestamp vector and add the final point on
	if (fixed_period != 0 && time != (points.size() + 1)*fixed_period)
	{
		convert_fixed_period_to_timestamps();
	}


	points.push_back(added_point);
	timestamps.push_back(time); 
	vecolities_calculated = false; //recalculate if needed

	return SUCCESS;

}

//******************** meaty, mathy functions*************************


// this is basically just vector adding each point in each array. The complexity comes from interpolation between points in time. 
// For this, we calculate the velocity along the path in order to derive position at any time.
Path Path::combine_paths(Path other_path, velocity my_initial_velocity, velocity their_initial_velocity)
{
	this->calculate_velocities_from_position(my_initial_velocity);
	other_path.calculate_velocities_from_position(their_initial_velocity);

	//  We start with the Path with the earliest timestamp, and go through by order of next timestamp.

	std::vector<timestamp> combined_timestamps;
	std::vector<point> combined_points;

	timestamp current_time, end_time;
	//true = this, false = other_path
	unsigned int my_index = 0, other_index = 0;
	
	if (this->get_time(0) == other_path.get_time(0))
	{
		current_time = this->get_time(0);
		my_index++;
		other_index++; //so we don't repeat this time
	}
	else if (this->get_time(0) < other_path.get_time(0))
	{
		current_time = this->get_time(0);
		my_index++;
	}
	else
	{
		current_time = other_path.get_time(0);
		other_index++;
	}
	end_time = std::max(this->get_time(this->points.size() - 1), other_path.get_time(other_path.points.size() - 1));

	

	while (current_time < end_time)
	{

		combined_points.push_back(this->get_position_at_time(current_time) + other_path.get_position_at_time(current_time));

		// advance to next time
		if (this->get_time(my_index) == other_path.get_time(other_index))
		{
			current_time = this->get_time(my_index++);
			other_index++; //so we don't repeat this time
		}
		else if (this->get_time(my_index) < other_path.get_time(other_index))
		{
			current_time = this->get_time(my_index++);
		}
		else
		{
			current_time = other_path.get_time(other_index++);
		}
	}
	Path ret(combined_points, combined_timestamps);
	return ret;
}

point Path::get_position_at_time(timestamp time)
{
	unsigned int index = get_index_from_time(time);
	timestamp time_difference = time - get_time(index);
	point ret = { points[index].x + (velocities[index].x * time_difference), points[index].y + (velocities[index].y * time_difference), points[index].z + (velocities[index].z * time_difference) };
	return ret;
	
}



velocity Path::get_velocity_from_finite_difference(point a, point b, timestamp dt)
{
	velocity ret = { (a.x - b.x) / dt, (a.y - b.y) / dt, (a.z - b.z) / dt };
	return ret;

}
// Implemented using centered finite difference
void Path::calculate_velocities_from_position(velocity initial_velocity)
{
	if (!vecolities_calculated)
	{
		velocities.clear();
		velocities.reserve(points.size());
		//first velocity is special case using initial velocity

		velocities.push_back(initial_velocity);

		velocity current_vel;
		unsigned int i;
		for (i = 1; i < points.size() - 1; ++i)
		{
			current_vel = get_velocity_from_finite_difference(points[i + 1],points[i - 1],(get_time(i + 1) - get_time(i - 1)));
			velocities.push_back(current_vel);
		}

		//final velocity is just a regular finite difference where i == points.size()-1
		assert(i == points.size() - 1);
		current_vel = get_velocity_from_finite_difference(points[i], points[i - 1], (get_time(i) - get_time(i - 1)));
		velocities.push_back(current_vel);

		vecolities_calculated = true;
	}
}

//*******************private/helper functions*************************

// copied from  
// http://stackoverflow.com/questions/446296/where-can-i-get-a-useful-c-binary-search-algorithm

template<class Iter, class T>
Iter binary_find(Iter begin, Iter end, T val)
{
	// Finds the lower bound in at most log(last - first) + 1 comparisons
	Iter i = std::lower_bound(begin, end, val);

	if (i != end && !(val < *i))
		return i; // found
	else
		return end; // not found
}

unsigned int Path::get_index_from_time(timestamp time)
{
	if (fixed_period != 0)
	{
		// implementation detail may be changed in future:
		// if fixed time interval between points, will truncate
		// due to integer division
		return time / fixed_period; 
	}
	else
	{
		// laughably, std::binary_search will only tell you if it exists in the vector
		// so we are using std::lower_bound instead to get an Iter then find the index based on it. This will give the closest index which is less than or equal.
		unsigned int index = std::distance(timestamps.begin(), std::lower_bound(timestamps.begin(), timestamps.end(), time));
		return index;
	}
}

void Path::convert_fixed_period_to_timestamps()
{
	if (fixed_period)
	{
		for (unsigned int i = 0; i < points.size(); ++i)
		{
			timestamps.push_back(fixed_period * i);
		}
		fixed_period = 0;
	}
}

//this function should simplify the constant period optimization
timestamp Path::get_time(unsigned int index)
{
	if (fixed_period)
	{
		return (index%points.size()) * fixed_period;
	}
	return timestamps[(index%points.size())];
}