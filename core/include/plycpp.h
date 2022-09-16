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

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <cassert>
#include <algorithm>
#include <typeindex>


namespace plycpp
{
	extern const std::type_index CHAR;
	extern const std::type_index UCHAR;
	extern const std::type_index SHORT;
	extern const std::type_index USHORT;
	extern const std::type_index INT;
	extern const std::type_index UINT;
	extern const std::type_index FLOAT;
	extern const std::type_index DOUBLE;

	enum FileFormat
	{
		ASCII,
		BINARY
	};

	//class Exception : public std::exception
	//{
	//public:
	//	Exception(const std::string& msg)
	//		: exception(msg.c_str())
	//	{}
	//};

	template<typename Key, typename T>
	struct KeyData
	{
		KeyData(const Key key, const T& data)
		{
			this->key = key;
			this->data = data;
		}

		Key key;
		T data;
	};

	/// A list of elements that can be accessed through a given key for convenience.
	/// Access by key is not meant to be fast, but practical.
	template<typename Key, typename Data>
	class IndexedList
	{
	private:
		typedef KeyData<Key, std::shared_ptr<Data> > MyKeyData;
		typedef std::vector<MyKeyData>  Container;
	public:
		typedef typename Container::iterator iterator;
		typedef typename Container::const_iterator const_iterator;

		std::shared_ptr<Data> operator[] (const Key& key)
		{
			auto it = std::find_if(begin(), end(), [&key](const MyKeyData& a){ return a.key == key; });
			if (it != end())
				return it->data;
			else {}
				//throw Exception("Invalid key.");
		}

		const std::shared_ptr<const Data> operator[] (const Key& key) const
		{
			auto it = std::find_if(begin(), end(), [&key](const MyKeyData& a){ return a.key == key; });
			if (it != end())
				return it->data;
			else {}
				//throw Exception("Invalid key.");
		}

		bool has_key(const Key &key)
		{
			auto it = std::find_if(begin(), end(), [&key](const MyKeyData &a) { return a.key == key; });
			if (it != end())
				return true;
			else
				return false;
		}

		void push_back(const Key& key, const std::shared_ptr<Data>& data)
		{
			container.push_back(MyKeyData(key, data));
		}

		void clear()
		{
			container.clear();
		}

		iterator begin() { return container.begin(); };
		const_iterator begin() const { return container.begin(); };
		iterator end() { return container.end(); };
		const_iterator end() const { return container.end(); };

	private:
		Container container;
	};

	class PropertyArray;
	class ElementArray;
	typedef std::shared_ptr<const PropertyArray> PropertyArrayConstPtr;
	typedef std::shared_ptr<PropertyArray> PropertyArrayPtr;
	typedef IndexedList<std::string, ElementArray > PLYData;

	class PropertyArray
	{
	public:
		PropertyArray(const std::type_index type, const size_t size, const bool isList = false);

		template<typename T>
		bool isOfType() const
		{
			return type == std::type_index(typeid(T));
		}

		template<typename T>
		const T* ptr() const
		{
			assert(isOfType<T>());
			return reinterpret_cast<const T*>(&data[0]);
		}

		template<typename T>
		T* ptr()
		{
			assert(isOfType<T>());
			return reinterpret_cast<T*>(data.data());
		}

		const size_t size() const
		{
			assert(data.size() % stepSize == 0);
			return data.size() / stepSize;
		}

		template<typename T>
		const T& at(const size_t i) const
		{
			assert(isOfType<T>());
			assert((i+1) * stepSize <= data.size());
			return  *reinterpret_cast<const T*>(&data[i * stepSize]);
		}

		template<typename T>
		T& at(const size_t i)
		{
			assert(isOfType<T>());
			assert((i + 1) * stepSize <= data.size());
			return  *reinterpret_cast<T*>(&data[i * stepSize]);
		}

		std::vector<unsigned char> data;
		const std::type_index type;
		const unsigned int stepSize;
		const bool isList = false;
	};

	class ElementArray
	{
	public:
		ElementArray(const size_t size)
			: size_(size)
		{}

		IndexedList<std::string, PropertyArray > properties;

		size_t size() const
		{
			return size_;
		}
	private:
		size_t size_;
	};

	/// Load PLY data
	void load(const std::string& filename, PLYData& data);

	/// Save PLY data
	void save(const std::string& filename, const PLYData& data, const FileFormat format = FileFormat::BINARY);

	/// Pack n properties -- each represented by a vector of type T --
	/// into a multichannel vector (e.g. of type vector<std::array<T, n> >)
	template<typename T, typename OutputVector>
	void packProperties(std::vector<std::shared_ptr<const PropertyArray> > properties, OutputVector& output)
	{
		output.clear();

		if (properties.empty() || !properties.front())
		{}
		//	throw Exception("Missing properties");

		const size_t size = properties.front()->size();
		const size_t nbProperties = properties.size();

		// Pointers to actual data
		std::vector<const T*> ptsData;
		for (auto& prop : properties)
		{
			// Check type consistency
			if (!prop || !prop->isOfType<T>() || prop->size() != size)
			{
				//throw Exception(std::string("Missing properties or type inconsistency. I was expecting data of type ") + typeid(T).name());
			}
			ptsData.push_back(prop->ptr<T>());
		}

		// Packing
		output.resize(size);
		for (size_t i = 0; i < size; ++i)
		{
			for (size_t j = 0; j < nbProperties; ++j)
			{
				output[i][j] = ptsData[j][i];
			}
		}
	}

	/// Unpack a multichannel vector into a list of properties.
	template<typename T, typename OutputVector>
	void unpackProperties(const OutputVector& cloud, std::vector < std::shared_ptr<PropertyArray> >& properties)
	{
		const size_t size = cloud.size();
		const size_t nbProperties = properties.size();

		// Initialize and get
		// Pointers to actual property data
		std::vector<T*> ptsData;
		for (auto& prop : properties)
		{
			prop.reset(new PropertyArray(std::type_index(typeid(T)), size));
			ptsData.push_back(prop->ptr<T>());
		}

		// Copy data
		for (size_t i = 0; i < size; ++i)
		{
			for (size_t j = 0; j < nbProperties; ++j)
			{
				ptsData[j][i] = cloud[i][j];
			}
		}
	}


	template<typename T, typename Cloud>
	void toPointCloud(const PLYData& plyData, Cloud& cloud)
	{
		cloud.clear();
		auto plyVertex = plyData["vertex"];
		if (plyVertex->size() == 0)
			return;
		std::vector<std::shared_ptr<const PropertyArray> > properties { plyVertex->properties["x"], plyVertex->properties["y"], plyVertex->properties["z"] };
		packProperties<T, Cloud>(properties, cloud);
	}

	template<typename T, typename Cloud>
	void toNormalCloud(const PLYData& plyData, Cloud& cloud)
	{
		cloud.clear();
		auto plyVertex = plyData["vertex"];
		if (plyVertex->size() == 0)
			return;
		std::vector<std::shared_ptr<const PropertyArray> > properties{ plyVertex->properties["nx"], plyVertex->properties["ny"], plyVertex->properties["nz"] };
		packProperties<T, Cloud>(properties, cloud);
	}

	template<typename T, typename Cloud>
	void fromPointCloud(const Cloud& points, PLYData& plyData)
	{
		const size_t size = points.size();

		plyData.clear();

		std::vector<std::shared_ptr<PropertyArray> > positionProperties(3);
		unpackProperties<T, Cloud>(points, positionProperties);

		std::shared_ptr<ElementArray> vertex(new ElementArray(size));
		vertex->properties.push_back("x", positionProperties[0]);
		vertex->properties.push_back("y", positionProperties[1]);
		vertex->properties.push_back("z", positionProperties[2]);

		plyData.push_back("vertex", vertex);
	}

	template<typename T, typename Cloud>
	void fromPointCloudAndNormals(const Cloud& points, const Cloud& normals, PLYData& plyData)
	{
		const size_t size = points.size();

		if (size != normals.size()) {}
			//throw Exception("Inconsistent size");


		plyData.clear();

		std::vector<std::shared_ptr<PropertyArray> > positionProperties(3);
		unpackProperties<T, Cloud>(points, positionProperties);

		std::vector<std::shared_ptr<PropertyArray> > normalProperties(3);
		unpackProperties<T, Cloud>(normals, normalProperties);

		std::shared_ptr<ElementArray> vertex(new ElementArray(size));
		vertex->properties.push_back("x", positionProperties[0]);
		vertex->properties.push_back("y", positionProperties[1]);
		vertex->properties.push_back("z", positionProperties[2]);

		vertex->properties.push_back("nx", normalProperties[0]);
		vertex->properties.push_back("ny", normalProperties[1]);
		vertex->properties.push_back("nz", normalProperties[2]);

		plyData.push_back("vertex", vertex);
	}

}
