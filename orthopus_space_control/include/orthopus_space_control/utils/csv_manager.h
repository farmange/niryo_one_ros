/*
 *  csv_manager.h
 *  Copyright (C) 2019 Orthopus
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CARTESIAN_CONTROLLER_CSV_MANAGERH
#define CARTESIAN_CONTROLLER_CSV_MANAGER_H

#include "ros/ros.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <string>

namespace space_control
{
class CSVManager
{
public:
  CSVManager(std::string filename)
  {
	// file pointer
	std::fstream fout;

	filename_ = filename;
	// opens an existing csv file or creates a new file.
	fout.open(filename_, std::ios::out | std::ios::app);
	if (!fout)
	{
	  ROS_ERROR_STREAM_NAMED("CSVManager", "Failed to open file " << filename_);
	  initialize_ = false;
	  return;
	}

	fout.close();
	initialize_ = true;
  };

  void get_records(std::vector<std::vector<std::string>> &records)
  {
	// File pointer
	std::fstream fin;
	if (!initialize_)
	{
	  ROS_ERROR_STREAM_NAMED("CSVManager", "Initialization issue, cannot get records.");
	  return;
	}

	// Open an existing file
	fin.open(filename_, std::ios::in);
	if (!fin)
	{
	  ROS_ERROR_STREAM_NAMED("CSVManager", "Failed to open file " << filename_);
	  initialize_ = false;
	  return;
	}

	// Read the Data from the file
	// as String Vector
	std::vector<std::string> row;
	std::string line, word, temp;
	// read an entire row and
	// store it in a string variable 'line'
	while (getline(fin, line))
	{
	  row.clear();

	  // used for breaking words
	  std::istringstream ss(line);
	  // read every column data of a row and
	  // store it in a string variable, 'word'
	  while (getline(ss, word, ','))
	  {
		// add all the column data
		// of a row to a vector
		row.push_back(word);
	  }
	  records.push_back(row);
	}
	fin.close();
  };

  void add_record(std::vector<std::string> &record)
  {
	// File pointer
	std::fstream fout;
	if (!initialize_)
	{
	  ROS_ERROR_STREAM_NAMED("CSVManager", "Initialization issue, cannot get records.");
	  return;
	}

	// opens an existing csv file or creates a new file.
	fout.open(filename_, std::ios::app);
	if (!fout)
	{
	  ROS_ERROR_STREAM_NAMED("CSVManager", "Failed to open file " << filename_);
	  return;
	}

	// Insert the data to file
	for (int i = 0; i < record.size(); i++)
	{
	  fout << record[i];
	  if (i < record.size() - 1)
	  {
		fout << ", ";
	  }
	  else
	  {
		fout << "\n";
	  }
	}
	fout.close();
  };

  void del_record(int record_number)
  {
	// File pointer
	std::fstream fout;

	clear_records();

	std::vector<std::vector<std::string>> init_records;
	CSVManager init_csv(filename_ + ".old");
	init_csv.get_records(init_records);

	for (int i = 0; i < init_records.size(); i++)
	{
	  if (i != record_number)
	  {
		add_record(init_records[i]);
	  }
	}
  };

  void clear_records()
  {
	// File pointer
	std::fstream fout;

	// opens an existing csv file or creates a new file.
	boost::filesystem::copy_file(filename_, filename_ + ".bak", boost::filesystem::copy_option::overwrite_if_exists);

	// opens and remove all content of the csv file.
	fout.open(filename_, std::ios::out);
	if (!fout)
	{
	  ROS_ERROR_STREAM_NAMED("CSVManager", "Failed to open file " << filename_);
	  return;
	}
	fout.close();
  };

protected:
private:
  bool initialize_;
  std::string filename_;
};
}
#endif
