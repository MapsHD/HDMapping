// MIT License
//
// Copyright(c) 2021 Romain Brégier
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <plycpp.h>

#include <fstream>
#include <sstream>
#include <unordered_map>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <typeindex>

namespace plycpp
{
	const std::type_index CHAR = std::type_index(typeid(int8_t));
	const std::type_index UCHAR = std::type_index(typeid(uint8_t));
	const std::type_index SHORT = std::type_index(typeid(int16_t));
	const std::type_index USHORT = std::type_index(typeid(uint16_t));
	const std::type_index INT = std::type_index(typeid(int32_t));
	const std::type_index UINT = std::type_index(typeid(uint32_t));
	const std::type_index FLOAT = std::type_index(typeid(float));
	const std::type_index DOUBLE = std::type_index(typeid(double));

	bool isBigEndianArchitecture()
	{
		/*union {
			uint32_t i;
			char c[4];
		} myunion = { 0x01020304 };
		return myunion.c[0] == 1;
		*/
		const uint32_t i = 0x01020304;
		return reinterpret_cast<const uint8_t*>(&i)[0] == 1;
	}



	const std::unordered_map<std::type_index, int> dataTypeByteSize{
		{ CHAR, sizeof(char) },
		{ UCHAR, sizeof(unsigned char) },
		{ SHORT, sizeof(int16_t) },
		{ USHORT, sizeof(uint16_t) },
		{ INT, sizeof(int32_t) },
		{ UINT, sizeof(uint32_t) },
		{ FLOAT, sizeof(float) },
		{ DOUBLE, sizeof(double) }
	};

	static_assert(sizeof(char) == 1, "Inconsistent type size");
	static_assert(sizeof(unsigned char) == 1, "Inconsistent type size");
	static_assert(sizeof(int16_t) == 2, "Inconsistent type size");
	static_assert(sizeof(uint16_t) == 2, "Inconsistent type size");
	static_assert(sizeof(int32_t) == 4, "Inconsistent type size");
	static_assert(sizeof(uint32_t) == 4, "Inconsistent type size");
	static_assert(sizeof(float) == 4, "Inconsistent type size");
	static_assert(sizeof(double) == 8, "Inconsistent type size");

	const std::unordered_map<std::string, std::type_index> strToDataType{
		{ "char", CHAR },
		{ "uchar", UCHAR },
		{ "short", SHORT },
		{ "ushort", USHORT },
		{ "int", INT },
		{ "int32", INT },
		{ "uint", UINT },
		{ "uint32", UINT },
		{ "float", FLOAT },
		{ "float32", FLOAT },
		{ "double", DOUBLE },
		{ "float64", DOUBLE }
	};

	const std::unordered_map<std::type_index, std::string> dataTypeToStr{
		{ CHAR, "char" },
		{ UCHAR, "uchar" },
		{ SHORT, "short" },
		{ USHORT, "ushort" },
		{ INT, "int" },
		{ UINT, "uint" },
		{ FLOAT, "float" },
		{ DOUBLE, "double" },
	};

	std::type_index parseDataType(const std::string& name)
	{
		const auto& it = strToDataType.find(name);
		if (it != strToDataType.end())
		{
			return it->second;
		}
		else{}
			//throw Exception(std::string("Unkown data type:" + name));
	}

	std::string dataTypeToString(const std::type_index& type)
	{
		auto it = dataTypeToStr.find(type);
		if (it == dataTypeToStr.end()){}
			//throw plycpp::Exception("Invalid data type");
		return it->second;
	}

	size_t dataTypeToStepSize(const std::type_index& type)
	{
		auto it = dataTypeByteSize.find(type);
		if (it == dataTypeByteSize.end()){}
			//throw plycpp::Exception("Invalid data type");
		return it->second;
	}


	PropertyArray::PropertyArray(const std::type_index type, const size_t size, const bool isList)
		: type(type),
		isList(isList),
		stepSize(dataTypeToStepSize(type))
	{
		this->data.resize(size * this->stepSize);
	}


	void splitString(const std::string& input, std::vector<std::string>& result)
	{
		result.clear();
		std::stringstream ss(input);
		while (true)
		{
			std::string elem;
			ss >> elem;
			if (ss.fail())
				break;
			result.push_back(elem);
		}

	}

	size_t strtol_except(const std::string& in)
	{
		std::stringstream ss(in);
		size_t val;
		ss >> val;
		if (ss.fail()){}
		//	throw Exception("Invalid unsigned integer");
		return val;
	}


	inline void readASCIIValue(std::ifstream& fin, unsigned char* const  ptData, const std::type_index& type)
	{
		int temp;
		if (type == CHAR)
		{
			fin >> temp;
			*reinterpret_cast<int8_t*>(ptData) = static_cast<int8_t>(temp);
		}
		else if (type == UCHAR)
		{
			fin >> temp;
			*reinterpret_cast<uint8_t*>(ptData) = static_cast<uint8_t>(temp);
		}
		else if (type == SHORT)
		{
			fin >> *reinterpret_cast<int16_t*>(ptData);
		}
		else if (type == USHORT)
		{
			fin >> *reinterpret_cast<uint16_t*>(ptData);
		}
		else if (type == INT)
		{
			fin >> *reinterpret_cast<int32_t*>(ptData);
		}
		else if (type == UINT)
		{
			fin >> *reinterpret_cast<uint32_t*>(ptData);
		}
		else if (type == FLOAT)
		{
			fin >> *reinterpret_cast<float*>(ptData);
		}
		else if (type == DOUBLE)
		{
			fin >> *reinterpret_cast<double*>(ptData);
		}
		else {}
			//throw Exception("Should not happen.");
	}

	inline void writeASCIIValue(std::ofstream& fout, unsigned char* const  ptData, const std::type_index type)
	{
		if (type == CHAR)
		{
			fout << int(*reinterpret_cast<int8_t*>(ptData));
		}
		else if (type == UCHAR)
		{
			fout << int(*reinterpret_cast<uint8_t*>(ptData));
		}
		else if (type == SHORT)
		{
			fout << *reinterpret_cast<int16_t*>(ptData);
		}
		else if (type == USHORT)
		{
			fout << *reinterpret_cast<uint16_t*>(ptData);
		}
		else if (type == INT)
		{
			fout << *reinterpret_cast<int32_t*>(ptData);
		}
		else if (type == UINT)
		{
			fout << *reinterpret_cast<uint32_t*>(ptData);
		}
		else if (type == FLOAT)
		{
			fout << *reinterpret_cast<float*>(ptData);
		}
		else if (type == DOUBLE)
		{
			fout << *reinterpret_cast<double*>(ptData);
		}
		else {}
			//throw Exception("Should not happen");
	}

	template <FileFormat format>
	void readDataContent(std::ifstream& fin, PLYData& data)
	{
		/// Store a pointer to the current place where to write next data for each property of each element
		std::unordered_map<PropertyArray*, unsigned char*> writingPlace;
		for (auto& elementTuple : data)
		{
			auto& element = elementTuple.data;
			for (auto& propertyTuple : element->properties)
			{
				auto& prop = propertyTuple.data;
				writingPlace[prop.get()] = prop->data.data();
			}
		}

		//// Iterate over elements array
		for (auto& elementArrayTuple : data)
		{
			auto& elementArray = elementArrayTuple.data;
			const size_t elementsCount = elementArray->size();
			// Iterate over elements
			for (size_t i = 0; i < elementsCount; ++i)
			{
				// Iterate over properties of the element
				for (auto& propertyTuple : elementArray->properties)
				{
					auto& prop = propertyTuple.data;

					if (!prop->isList)
					{
						// Read data
						auto& ptData = writingPlace[prop.get()];
						// Safety check
						assert(ptData >= prop->data.data());
						assert(ptData + prop->stepSize <= prop->data.data() + prop->data.size());

						if (format == ASCII)
						{
							readASCIIValue(fin, ptData, prop->type);

						}
						else
						{
							fin.read(reinterpret_cast<char*>(ptData), prop->stepSize);
						}
						// Increment
						ptData += prop->stepSize;
					}
					else
					{
						// Read count
						uint8_t count;

						if (format == ASCII)
						{
							int temp;
							fin >> temp;
							count = static_cast<unsigned char>(temp);
						}
						else
						{
							fin.read(reinterpret_cast<char*>(&count), sizeof(count));
						}
						if (fin.fail() || count != 3)
						{
							//throw Exception("Only lists of 3 values are supported");
						}

						// Read data
						auto& ptData = writingPlace[prop.get()];
						const size_t chunkSize = 3 * prop->stepSize;

						// Safety check
						assert(ptData >= prop->data.data());
						assert(ptData + chunkSize <= prop->data.data() + prop->data.size());

						if (format == ASCII)
						{
							readASCIIValue(fin, ptData, prop->type);
							ptData += prop->stepSize;
							readASCIIValue(fin, ptData, prop->type);
							ptData += prop->stepSize;
							readASCIIValue(fin, ptData, prop->type);
							ptData += prop->stepSize;
						}
						else
						{
							fin.read(reinterpret_cast<char*>(ptData), chunkSize);
							ptData += chunkSize;
						}
					}
				}
			}
		}
	}

	void myGetline(std::ifstream& fin, std::string& line)
	{
		std::getline(fin, line);
		// Files created with Windows have a carriage return
		if (!line.empty() && line.back() == '\r')
			line.pop_back();
	}

	void load(const std::string& filename, PLYData& data)
	{
		// Read header and reserve memory
		data.clear();
		std::string format;
		std::string version;

		std::ifstream fin(filename, std::ios::binary);
		//fin.sync_with_stdio(false);

		if (!fin.is_open()){}
			//throw Exception(std::string("Unable to open ") + filename);

		std::string line;
		myGetline(fin, line);

		std::shared_ptr<ElementArray> currentElement = nullptr;

		if (line != "ply")
		{
			//throw Exception("Missing magic number ""ply""");
		}

		while (line != "end_header")
		{
			myGetline(fin, line);
			if (fin.fail()){}
				//throw Exception("Header parsing exception");

			std::vector<std::string> lineContent;
			splitString(line, lineContent);

			if (lineContent.size() == 3 && lineContent[0] == "format")
			{
				format = lineContent[1];
				version = lineContent[2];
			}
			if (lineContent.size() == 3 && lineContent[0] == "element")
			{
				// New element
				const std::string& name = lineContent[1];
				const size_t count = strtol_except(lineContent[2]);

				currentElement.reset(new ElementArray(count));

				data.push_back(name, currentElement);
			}
			else if (lineContent.size() == 3 && lineContent[0] == "property")
			{
				if (!currentElement){}
					//throw Exception("Header issue!");

				// New property
				const std::type_index dataType = parseDataType(lineContent[1]);
				const std::string& name = lineContent[2];

				std::shared_ptr<PropertyArray> newProperty(new PropertyArray(dataType, currentElement->size()));
				currentElement->properties.push_back(name, newProperty);
			}
			else if (lineContent.size() == 5 && lineContent[0] == "property" && lineContent[1] == "list")
			{
				if (!currentElement){}
					//throw Exception("Header issue!");

				const std::type_index indexCountType = parseDataType(lineContent[2]);
				const std::type_index dataType = parseDataType(lineContent[3]);
				const std::string& name = lineContent[4];

				if (indexCountType != UCHAR){}
					//throw Exception("Only uchar is supported as counting type for lists");

				std::shared_ptr<PropertyArray> newProperty(new PropertyArray(dataType, 3 * currentElement->size(), true));
				currentElement->properties.push_back(name, newProperty);
			}

		}

		if (fin.fail())
		{
			//throw Exception("Issue while parsing header");
		}


		/////////////////////////////////////
		// Read data
		if (format == "ascii")
		{
			readDataContent<FileFormat::ASCII>(fin, data);

			if (fin.fail())
			{
				//throw Exception("Issue while parsing ascii data");
			}
		}
		else
		{
			const bool isBigEndianArchitecture_ = isBigEndianArchitecture();
			if (format != "binary_little_endian" && format != "binary_big_endian"){}
				//throw Exception("Unknown binary format");


			if ((isBigEndianArchitecture_ && format != "binary_big_endian")
				|| (!isBigEndianArchitecture_ && format != "binary_little_endian")){}
				//throw Exception("Endianness conversion is not supported yet");

			readDataContent<FileFormat::BINARY>(fin, data);

			if (fin.fail())
			{
				//throw Exception("Issue while parsing binary data");
			}

			// Ensure we reached the end of file by trying to read a last char
			char useless;
			fin.read(&useless, 1);
			if (!fin.eof())
			{
				//throw Exception("End of file not reached at the end of parsing.");
			}
		}
	}


	template<FileFormat format>
	void writeDataContent(std::ofstream& fout, const PLYData& data)
	{
		/// Store a pointer to the current place from which to read next data for each property of each element
		std::unordered_map<PropertyArray*, unsigned char*> readingPlace;
		for (auto& elementTuple : data)
		{
			auto& element = elementTuple.data;
			for (auto& propertyTuple : element->properties)
			{
				auto& prop = propertyTuple.data;
				readingPlace[prop.get()] = prop->data.data();
			}
		}

		//// Iterate over elements array
		for (auto& elementArrayTuple : data)
		{
			auto& elementArray = elementArrayTuple.data;
			const size_t elementsCount = elementArray->size();
			// Iterate over elements
			for (size_t i = 0; i < elementsCount; ++i)
			{
				// Iterate over properties of the element
				for (auto& propertyTuple : elementArray->properties)
				{
					auto& prop = propertyTuple.data;
					// Write data
					auto& ptData = readingPlace[prop.get()];
					if (!prop->isList)
					{
						// Safety check
						assert(ptData >= prop->data.data());
						assert(ptData + prop->stepSize <= prop->data.data() + prop->data.size());
						if (format == FileFormat::BINARY)
							fout.write(reinterpret_cast<const char*>(ptData), prop->stepSize);
						else
						{
							writeASCIIValue(fout, ptData, prop->type);
							fout << " ";
						}
						ptData += prop->stepSize;
					}
					else
					{
						if (format == FileFormat::BINARY)
						{
							const unsigned char count = 3;
							// Write the number of elements
							fout.write(reinterpret_cast<const char*>(&count), sizeof(unsigned char));
							// Write data
							const size_t chunckSize = 3 * prop->stepSize;
							// Safety check
							assert(ptData >= prop->data.data());
							assert(ptData + chunckSize <= prop->data.data() + prop->data.size());
							fout.write(reinterpret_cast<const char*>(ptData), chunckSize);
							ptData += chunckSize;
						}
						else
						{
							fout << "3 ";
							writeASCIIValue(fout, ptData, prop->type);
							fout << " ";
							ptData += prop->stepSize;
							writeASCIIValue(fout, ptData, prop->type);
							fout << " ";
							ptData += prop->stepSize;
							writeASCIIValue(fout, ptData, prop->type);
							fout << " ";
							ptData += prop->stepSize;
						}

					}


				}
				if (format == FileFormat::ASCII)
				{
					fout << "\n";
				}
			}
		}
	}

	void save(const std::string& filename, const PLYData& data, const FileFormat format)
	{
		std::ofstream fout(filename, std::ios::binary);

		// Write header
		fout << "ply\n";
		switch (format)
		{
		case FileFormat::ASCII:
			fout << "format ascii 1.0\n";
			break;
		case FileFormat::BINARY:
			if (isBigEndianArchitecture())
				fout << "format binary_big_endian 1.0\n";
			else
				fout << "format binary_little_endian 1.0\n";
			break;
		default:
			//throw Exception("Unknown file format. Should not happen.");
			break;
		}

		// Iterate over elements array
		for (const auto& elementArrayTuple : data)
		{
			const auto& elementArrayName = elementArrayTuple.key;
			auto& elementArray = elementArrayTuple.data;
			const size_t elementsCount = elementArray->size();

			fout << "element " << elementArrayName << " " << elementsCount << std::endl;
			// Iterate over properties
			for (const auto& propertyTuple : elementArray->properties)
			{
				auto& propName = propertyTuple.key;
				auto& prop = propertyTuple.data;

				if (!prop){}
					//throw Exception("Null property " + elementArrayName + " -- " + propName);

				// String name of the property type
				const auto& itTypeName = dataTypeToStr.find(prop->type);
				if (itTypeName == dataTypeToStr.end()){}
					//throw Exception("Should not happen");

				if (!prop->isList)
				{
					if (prop->data.size() != elementsCount * prop->stepSize)
					{
						//throw Exception("Inconsistent size for " + elementArrayName + " -- " + propName);
					}

					fout << "property " << itTypeName->second << " " << propName << std::endl;
				}
				else
				{
					if (prop->data.size() != 3 * elementsCount * prop->stepSize)
					{
						//throw Exception("Inconsistent size for list " + elementArrayName + " -- " + propName);
					}

					fout << "property list uchar " << itTypeName->second << " " << propName << std::endl;
				}

			}
		}
		fout << "end_header" << std::endl;

		// Write data
		switch (format)
		{
		case FileFormat::BINARY:
			writeDataContent<FileFormat::BINARY>(fout, data);
			break;
		case FileFormat::ASCII:
			writeDataContent<FileFormat::ASCII>(fout, data);
			break;
		default:
			//throw Exception("Unknown file format. Should not happen.");
			break;
		}


		if (fout.fail())
		{
			//throw Exception("Problem while writing binary data");
		}
	}
}
