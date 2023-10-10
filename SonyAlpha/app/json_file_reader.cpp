#ifndef __boost_json_files
#define __boost_json_files
// ======================================================================================================
//
// Library for saving the camera settings to a json file which may be read back in if requested to do so 
// by the .ini file
//
// ======================================================================================================
#include <iostream>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/format.hpp>
#include <string>

#include "ini_file_reader.cpp"

#include "focus_adjustment.h"

// defines the name of the json file which stores the current settings for potential recall
//
std::string f_name = "/home/anthony/filename.json";

// ======================================================================================================
// 
// write the values to json file
//
// ======================================================================================================
#if defined(__focus_adjustment)
int write_camsets_to_json(std::uint32_t aper, std::uint32_t iso, std::uint32_t sc, std::uint32_t fm, std::uint32_t fa, std::uint32_t wb, std::uint32_t expro, std::uint32_t ss, std::int32_t nf) {
#else
int write_camsets_to_json(std::uint32_t aper, std::uint32_t iso, std::uint32_t sc, std::uint32_t fm, std::uint32_t fa, std::uint32_t wb, std::uint32_t expro, std::uint32_t ss) {
#endif
    // 
    // print the expected string the the standard out for DEBUG 
    //
#if defined(__focus_adjustment)
    std::cout << boost::format("{\"s_aperture\":%1%, \"s_iso\": %2%, \"s_still_cap\" : %3%, \"s_foc_mode\" : %4%, \"s_foc_area\" : %5%, \"s_white_bal\" : %6%, \"s_exp_pro\" : %7%, \"s_shut_spd\" : %8%, \"s_near_far\" : %9% }") % aper % iso % sc % fm % fa % wb % expro % ss % nf << std::endl;
    std::stringstream s1;
    s1 << boost::format("{\"s_aperture\":%1%, \"s_iso\": %2%, \"s_still_cap\" : %3%, \"s_foc_mode\" : %4%, \"s_foc_area\" : %5%, \"s_white_bal\" : %6%, \"s_exp_pro\" : %7%, \"s_shut_spd\" : %8%, \"s_near_far\" : %9% }") % aper % iso % sc % fm % fa % wb % expro % ss % nf;
#else
    std::cout << boost::format("{\"s_aperture\":%1%, \"s_iso\": %2%, \"s_still_cap\" : %3%, \"s_foc_mode\" : %4%, \"s_foc_area\" : %5%, \"s_white_bal\" : %6%, \"s_exp_pro\" : %7%, \"s_shut_spd\" : %8% }") % aper % iso % sc % fm % fa % wb % expro % ss << std::endl;
    std::stringstream s1;
    s1 << boost::format("{\"s_aperture\":%1%, \"s_iso\": %2%, \"s_still_cap\" : %3%, \"s_foc_mode\" : %4%, \"s_foc_area\" : %5%, \"s_white_bal\" : %6%, \"s_exp_pro\" : %7%, \"s_shut_spd\" : %8% }") % aper % iso % sc % fm % fa % wb % expro % ss;
#endif
    //
    // create the boost property tree for writing
    //
    boost::property_tree::ptree pro_tree;
    //
    // copy the stringstream to the property tree
    //
	cli::tout << "\033[31m STRINGSTREAM \033[0m" << s1.str() << " NF " << nf << std::endl;

	try {
        boost::property_tree::read_json(s1, pro_tree);
    } catch (boost::property_tree::json_parser_error& e) {
        std::wcout << L"\033[34;41m ERROR : \033[0m" << e.what() << std::endl;
        return -1;
	}
	//try {
    //    boost::property_tree::read_json(s1, pro_tree);
    //
    //} catch (boost::property_tree::json_parser_error& e) {
    //    std::wcout << L"ERROR : " << e.what() << std::endl;
    //    exit(-1);
    //}
	//cli::tout << "copied string stream" << std::endl;	
	
    //
    // add any extra ones here (these come from the .ini file)
    //
    // read the xml file and preserve this settings
    //
    int xml_def_on = get_ini_int_component("sony", "xml_defaults");
    pro_tree.add<int>("xml_defaults", xml_def_on);
	cli::tout << "added to pro tree a tag" << std::endl;
    //
    // DEBUG write the result to the std output
    //
    boost::property_tree::write_json(std::cout, pro_tree, false);
	cli::tout << "wrote to standard out" << std::endl;
    //
    // write the result to the named file
    //
    boost::property_tree::write_json(f_name, pro_tree);
	cli::tout << "wrote to file" << std::endl;
    return 1;
}

//
// •••••••••••• function to write settings to json seemed to be issue with some values in format print so tried adding all of them separately ••••••••••••
//              slightly slower than the routine above
//
#if defined(__focus_adjustment)
int write_camsets_to_json2(std::uint32_t aper, std::uint32_t iso, std::uint32_t sc, std::uint32_t fm, std::uint32_t fa, std::uint32_t wb, std::uint32_t expro, std::uint32_t ss, std::int32_t nf) {
#else
int write_camsets_to_json2(std::uint32_t aper, std::uint32_t iso, std::uint32_t sc, std::uint32_t fm, std::uint32_t fa, std::uint32_t wb, std::uint32_t expro, std::uint32_t ss) {
#endif
    // 
    // print the expected string the the standard out for DEBUG / INFO
    //
#if defined(__focus_adjustment)
    std::cout << boost::format("{\"s_aperture\":%1%, \"s_iso\": %2%, \"s_still_cap\" : %3%, \"s_foc_mode\" : %4%, \"s_foc_area\" : %5%, \"s_white_bal\" : %6%, \"s_exp_pro\" : %7%, \"s_shut_spd\" : %8%, \"s_near_far\" : %9% }") % aper % iso % sc % fm % fa % wb % expro % ss % nf << std::endl;
#else
    std::cout << boost::format("{\"s_aperture\":%1%, \"s_iso\": %2%, \"s_still_cap\" : %3%, \"s_foc_mode\" : %4%, \"s_foc_area\" : %5%, \"s_white_bal\" : %6%, \"s_exp_pro\" : %7%, \"s_shut_spd\" : %8% }") % aper % iso % sc % fm % fa % wb % expro % ss << std::endl;
#endif
    //
    // create the boost property tree for writing
    //
    boost::property_tree::ptree pro_tree;
	
    //
    // add any extra ones here (these come from the .ini file)
    //
    // read the xml file and preserve this settings
    //
    int xml_def_on = get_ini_int_component("sony", "xml_defaults");
    pro_tree.add<int>("xml_defaults", xml_def_on);
	cli::tout << "added to pro tree a tag" << std::endl;
    pro_tree.add<std::uint32_t>("s_aperture", aper);	
    pro_tree.add<std::uint32_t>("s_still_cap", iso);
    pro_tree.add<std::uint32_t>("s_iso", sc);
    pro_tree.add<std::uint32_t>("s_foc_mode", fm);
    pro_tree.add<std::uint32_t>("s_foc_area", fa);	
    pro_tree.add<std::uint32_t>("s_white_bal", wb);	
    pro_tree.add<std::uint32_t>("s_exp_pro", expro);
    pro_tree.add<std::uint32_t>("s_shut_spd", ss);
	
    //
    // DEBUG write the result to the std output
    //
    boost::property_tree::write_json(std::cout, pro_tree, false);
	cli::tout << "wrote to standard out" << std::endl;
    //
    // write the result to the named file
    //
    boost::property_tree::write_json(f_name, pro_tree);
	cli::tout << "wrote to file" << std::endl;
    return 1;
}
// ======================================================================================================
// 
// read the values to json file
//
// ======================================================================================================
#if defined(__focus_adjustment)
std::tuple< int, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::int16_t > read_camsets_from_json(void) {
#else
std::tuple< int, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t > read_camsets_from_json(void) {
#endif
    //
    // Create a root property tree to read into
    //
    boost::property_tree::ptree root;

    //
    // Load the json file in this ptree
    //
    boost::property_tree::read_json(f_name, root);
    //	
    // dump the entire read to the standard output
    //
    std::stringstream root_full_read;
    boost::property_tree::write_json(root_full_read, root, true);
    std::cout << "The raw incoming json_file stream is :: \n" << root_full_read.str() << std::endl;
    //
    // read each individual key
    //	
    std::uint32_t aper = root.get<std::uint32_t>("s_aperture", 0);
    std::uint32_t iso = root.get<std::uint32_t>("s_iso", 0);
    std::uint32_t sc = root.get<std::uint32_t>("s_still_cap", 0);
    std::uint32_t fm = root.get<std::uint32_t>("s_foc_mode", 0);
    std::uint32_t fa = root.get<std::uint32_t>("s_foc_area", 0);
    std::uint32_t wb = root.get<std::uint32_t>("s_white_bal", 0);
    std::uint32_t expro = root.get<std::uint32_t>("s_exp_pro", 0);
    std::uint32_t ss = root.get<std::uint32_t>("s_shut_spd", 0);
#if defined(__focus_adjustment)
    std::int16_t nf = root.get<std::uint32_t>("s_near_far", 0);
#endif

    auto x_def = root.get<int>("xml_defaults", 0);

    std::cout << "xml_defaults from the ini file had value " << x_def << std::endl;
#if defined(__focus_adjustment)
    return std::make_tuple(x_def, aper, iso, sc, fm, fa, wb, expro, ss, nf);
#else
    return std::make_tuple(x_def, aper, iso, sc, fm, fa, wb, expro, ss);
#endif
}

//
// Shows how to read and write a formatted string to a json tree
//
void read_json_data_from_string_test(void)
{

    std::string str_json = "{\"count\":10,\"people\":[{ \"firstName\": \"Brett\", \"lastName\":\"McLaughlin\", \"email\": \"aaaa\" },{ \"firstName\": \"Jason\", \"lastName\":\"Hunter\", \"email\": \"bbbb\"},{ \"firstName\": \"Elliotte\", \"lastName\":\"Harold\", \"email\": \"cccc\" }]}";
    std::stringstream str_stream(str_json);
    boost::property_tree::ptree root;
    boost::property_tree::read_json(str_stream,root);
    root.put("upid","001");

    boost::property_tree::ptree exif_array;
    boost::property_tree::ptree array1, array2, array3;
    array1.put("Make", "NIKON");
    array2.put("DateTime", "2011:05:31 06:47:09");
    array3.put("Software", "Ver.1.01");

    //   exif_array.push_back(std::make_pair("Make", "NIKON"));
    //   exif_array.push_back(std::make_pair("DateTime", "2011:05:31 06:47:09"));
    //   exif_array.push_back(std::make_pair("Software", "Ver.1.01"));

    exif_array.push_back(std::make_pair("", array1));
    exif_array.push_back(std::make_pair("", array2));
    exif_array.push_back(std::make_pair("", array3));

    root.put_child("exifs", exif_array);
    std::stringstream str_stream_temp;
    boost::property_tree::write_json(str_stream_temp, root);
    //write_json(str_stream_temp, root);
    std::string str = str_stream_temp.str();
    cout<<str<<endl;
}

void read_json_data_from_string(std::string str_json)
{
    std::stringstream str_stream(str_json);
    boost::property_tree::ptree root;
    boost::property_tree::read_json(str_stream,root);
    root.put("upid","001");

    boost::property_tree::ptree exif_array;
    boost::property_tree::ptree array1, array2, array3;
    array1.put("Make", "NIKON");
    array2.put("DateTime", "2011:05:31 06:47:09");
    array3.put("Software", "Ver.1.01");

    //   exif_array.push_back(std::make_pair("Make", "NIKON"));
    //   exif_array.push_back(std::make_pair("DateTime", "2011:05:31 06:47:09"));
    //   exif_array.push_back(std::make_pair("Software", "Ver.1.01"));

    exif_array.push_back(std::make_pair("", array1));
    exif_array.push_back(std::make_pair("", array2));
    exif_array.push_back(std::make_pair("", array3));

    root.put_child("exifs", exif_array);
    std::stringstream str_stream_temp;
    boost::property_tree::write_json(str_stream_temp, root);
    //write_json(str_stream_temp, root);
    std::string str = str_stream_temp.str();
    cout<<str<<endl;
}

void bianli_json(void)
{
    string json_string="{\"-f\": \"/usr/reservedfff_dir\", \"-s\": \"/usr/reservedddd_dir\"}";

    string str_head;
    string str_node_val;
    boost::property_tree::ptree pt,p1,p2;

    stringstream stream(json_string);
    try
    {
        boost::property_tree::read_json<boost::property_tree::ptree>(stream, pt);
        cout<<"parsing ok\n"<<endl;
        for (boost::property_tree::ptree::iterator ita = pt.begin(); ita != pt.end(); ++ita)
        {
            cout<<"first:"<<ita->first<<endl;
            str_node_val = pt.get<string>(ita->first);
            cout<<str_node_val<<endl;
        }
    }
    catch(std::runtime_error& e)
    {
        std::cout<<e.what()<<endl;
    }
}

std::string write_json_data_into_a_string(boost::property_tree::ptree item)
{
    std::stringstream is;
    boost::property_tree::write_json(is,item);
    std::string s = is.str();
    cout<<"json s:"<<s<<endl;
	return s;
}

void write_json_data_into_file(std::string file_path)
{
    printf("%s\n","write json data");

    boost::property_tree::ptree root, items;

    boost::property_tree::ptree item1;
    item1.put("ID","1");
    item1.put("Name","sony");
    items.push_back(std::make_pair("1",item1));

    boost::property_tree::ptree item2;
    item2.put("ID","2");
    item2.put("Name","micasense");
    items.push_back(std::make_pair("2",item2));

    boost::property_tree::ptree item3;
    item3.put("ID","3");
    item3.put("Name","workswell");
    items.push_back(std::make_pair("2",item3));
	
    root.put_child("users",items);
    boost::property_tree::write_json(file_path, root);
}

//
// example below is for unit test and usage of the library
//
//#define __do_Test
#ifdef __do_Test
int main(int argc, const char* argv[]) {
#if defined(__focus_adjustment)
    int ret = 0;
    ret = write_camsets_to_json(34, 111, 102, 4, 5, 17, 5, 1);
    std::tuple< int, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t > json_we_read = read_camsets_from_json();

    std::cout << "xml_option " << std::get<0>(json_we_read) << "\n";
    std::cout << "Aperture " << std::get<1>(json_we_read) << "\n";
    std::cout << "Iso " << std::get<2>(json_we_read) << "\n";
    std::cout << "Still Cap Mode " << std::get<3>(json_we_read) << "\n";
    std::cout << "Focus Mode " << std::get<4>(json_we_read) << "\n";
    std::cout << "Focus Area " << std::get<5>(json_we_read) << "\n";
    std::cout << "White Bal " << std::get<6>(json_we_read) << "\n";
    std::cout << "Exposure Program Mode " << std::get<7>(json_we_read) << "\n";
    std::cout << "Shutter Speed " << std::get<8>(json_we_read) << "\n";
    std::cout << "Near Far " << std::get<9>(json_we_read) << "\n";
#else
    int ret = 0;
    ret = write_camsets_to_json(34, 111, 102, 4, 5, 17, 5, 1);
    std::tuple< int, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t > json_we_read = read_camsets_from_json();

    std::cout << "xml_option " << std::get<0>(json_we_read) << "\n";
    std::cout << "Aperture " << std::get<1>(json_we_read) << "\n";
    std::cout << "Iso " << std::get<2>(json_we_read) << "\n";
    std::cout << "Still Cap Mode " << std::get<3>(json_we_read) << "\n";
    std::cout << "Focus Mode " << std::get<4>(json_we_read) << "\n";
    std::cout << "Focus Area " << std::get<5>(json_we_read) << "\n";
    std::cout << "White Bal " << std::get<6>(json_we_read) << "\n";
    std::cout << "Exposure Program Mode " << std::get<7>(json_we_read) << "\n";
    std::cout << "Shutter Speed " << std::get<8>(json_we_read) << "\n";
#endif
}
#endif // test code

#endif
