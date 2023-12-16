//   example webserver in C++ using crow and mustache bbs message board and other methods are shown
//
//    config.json
//    ========================
//    "db_host": "localhost",
//    "db_user": "mysqluser",
//    "db_pass": "mysqlpass",
//    "db_name": "bbs"

//    html mustache = bbs.html
//    ========================
//<!DOCTYPE html>
//<html lang="en">
//  <head>
//    <meta charset="UTF-8">
//    <title>This is the ACP BBS</title>
//      <style type="text/css" media="screen, handheld, print, tv" >
//      <!--
//      .b3-2 h2, .b3-2 h3, .b3-2 p { border: solid; border-width: 2px 4px 4px 2px;
//      padding: 5px; margin: 0px 0px 4px; }
//      .b3-2 h2 { border-color: blue; border-style: dashed; }
//      .b3-2 h3 { border-top-color: lightgreen; border-right-color: plum;
//      border-bottom-color: plum; border-left-color: lightgreen; border-style: double; border-top-style: solid; border-width: thick }
//      .b3-2 p { border-color: mediumpurple transparent;  }
//      -->
//      </style>
//  </head>
//  <body>
//    <div class=b3-2>
//    <h3>ACP Builitin Board</h3>
//    <ul>
//      {{# posts}}
//      <li>{{id}}: {{text}}</li>
//      {{/ posts}}
//    </ul>
//    <p>Type you're message here</p>
//    <form action="/post" method="post">
//      <input type="text" name="text"><input type="submit">
//    </form>
//	</div>
//  </body>
//</html>
#include <iostream>
#include <string>
	
#include <memory>
#include <mysql_connection.h>
#include <mysql_driver.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>
#include "crow_all.h"
#include "crow.h"
#include "cpplinq.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include <chrono>
#include "fmt/format.h"
#include <fmt/chrono.h>
#include <fmt/time.h>

struct date {
    int y, m, d;

    make_date_str(int y, int m, int d)
        : y(y), m(m), d(d)
    {}

    std::ostream &operator<<(std::ostream &o, date const&rhs)
    {
        return o << rhs.y << '/' << rhs.m << '/' << rhs.d;
    }
};

int
main() {
  std::ifstream conf("config.json");
  if (!conf) {
    std::cerr << "config.json not found" << std::endl;
    return 1;
  }
  std::string json = {
    std::istreambuf_iterator<char>(conf),
    std::istreambuf_iterator<char>()};
  crow::json::rvalue config = crow::json::load(json);

  auto driver = sql::mysql::get_mysql_driver_instance();
  auto raw_con = driver->connect(
    (std::string) config["db_host"].s(),
    (std::string) config["db_user"].s(),
    (std::string) config["db_pass"].s());
  auto con = std::unique_ptr<sql::Connection>(raw_con);
  con->setSchema((std::string) config["db_name"].s());

  crow::SimpleApp app;
  crow::mustache::set_base(".");

  // if you want to create it when you start this program uncomment this
  //
  // auto st = " CREATE TABLE bbs (
  //            id INT NOT NULL AUTO_INCREMENT PRIMARY KEY,
  //            text VARCHAR(100),
  //            created TIMESTAMP DEFAULT NOW()
  //            ) ENGINE=InnoDB DEFAULT CHARSET=utf8;";
  // auto stmt = std::unique_ptr<sql::PreparedStatement>(
  //    con->prepareStatement(st));
  // stmt->executeUpdate();			  

  CROW_ROUTE(app, "/")
  ([&]{
    auto stmt = std::unique_ptr<sql::PreparedStatement>(
      con->prepareStatement("select * from bbs order by created"));
    auto res = std::unique_ptr<sql::ResultSet>(
      stmt->executeQuery());
    int n = 0;
    crow::mustache::context ctx;
    while (res->next()) {
      ctx["posts"][n]["id"] = res->getInt("id");
      ctx["posts"][n]["text"] = res->getString("text");
      n++;
    } 
    return crow::mustache::load("bbs.html").render(ctx);
  });

  CROW_ROUTE(app, "/post")
      .methods("POST"_method)
  ([&](const crow::request& req, crow::response& res){
    crow::query_string params(req.body);
    auto stmt = std::unique_ptr<sql::PreparedStatement>(
      con->prepareStatement("insert into bbs(text) values(?)"));
    stmt->setString(1, params.get("text"));
    stmt->executeUpdate();
    res = crow::response(302);
    res.set_header("Location", "/");
    res.end();
  });

  // simple json response get the config credentials
  CROW_ROUTE(app, "/json_data")
  ([&]{
        crow::json::wvalue x;
        x["db_pass"] = (std::string) config["db_pass"].s();
        x["db_host"] = (std::string) config["db_host"].s();
        x["db_name"] = (std::string) config["db_name"].s();
        x["db_user"] = (std::string) config["db_user"].s();
        return x;
  });
  
  CROW_ROUTE(app, "/sumfun/<int>")
  ([&] (int count){
       using namespace cpplinq;
       int ints[] = {3,1,4,1,5,10,12,9,2,6,5,4,12,66,99,20,21,65,67,76,34,75,18};

       // Computes the sum of all chosen numbers in the sequence above
       auto x =
            from_array(ints)
        >>  where ([](int i) {return i % count == 0;})     // Keep only numbers with the modulus of the arguemnt passed
        >>  sum ()                                         // Sum those numbers
        ;
        std::ostringstream os;
        os << x << "You selected from our memory array! : ";
        return crow::response(os.str());
  });

  CROW_ROUTE(app, "/params")
  ([&](const crow::request& req){
          std::ostringstream os;
          os << "Params: " << req.url_params << "\n\n"; 
          os << "The key 'my_key' was " << (req.url_params.get("my_key") == nullptr ? "not " : "") << "found.\n";
          if(req.url_params.get("get_this_double") != nullptr) {
              double countD = boost::lexical_cast<double>(req.url_params.get("get_this_double"));
              os << "The value of 'get_this_value' is " <<  countD << '\n';
          }
          auto count = req.url_params.get_list("value_list");
          os << "The key 'count' contains " << count.size() << " value(s).\n";
          for(const auto& countVal : count) {
              os << " - " << countVal << '\n';
          }
          return crow::response{os.str()};
   });

   //      * curl {ip}:40081/mul/4/7 
   CROW_ROUTE(app, "/mul/<int>/<int>")
   ([&](const crow::request& req, crow::response& res, int a, int b){
        std::ostringstream os;
        os << a*b;
        res.write(os.str());
        res.end();
   });

   //      * curl -d '{"add1":13,"add2":7}' {ip}:40081/add_json  
   CROW_ROUTE(app, "/add_json")
        .methods(crow::HTTPMethod::POST)
   ([&](const crow::request& req){
        auto x = crow::json::load(req.body);
        if (!x)
            return crow::response(400);
        auto sum = x["add1"].i()+x["add2"].i();
        std::ostringstream os;
        os << sum;
        return crow::response{os.str()};
   });

   CROW_ROUTE(app, "/getLatLong")
        .methods("GET"_method)
   ([&](const crow::request& req){
        crow::json::wvalue x;
        auto params = req.url_params;
        auto param_longitude = atof(params.get("longitude"));
        auto param_latitude = atof(params.get("latitude"));
        x["lat"] = param_latitude;
        x["lon"] = param_longitude;
        return x;
   });

   //      * curl {ip}:40081/mustache_page 
   CROW_ROUTE(app, "/mustashe_page")
   ([&]{
        auto page = crow::mustache::load("mustache.html");
        return page.render();
   });
   
   //      * curl -d '{"year":2023,"month":7,"day":12}' {ip}:40081/date_json  
   CROW_ROUTE(app, "/date_json")
        .methods(crow::HTTPMethod::POST)
   ([&](const crow::request& req){
        auto x = crow::json::load(req.body);
        if (!x)
            return crow::response(400);
        auto yr = x["year"].i();
		auto mon = x["month"].i();
		auto dy = x["day"].i();
        std::string fmt_date = fmt::format("Date sent is {}", make_date_str{dy, mon, yr});
        return crow::response{fmt_date};
   });

   //      * curl {ip}:40081/date_time_server  
   CROW_ROUTE(app, "/date_time_server")
        .methods(crow::HTTPMethod::POST)
   ([&]{
	    std::ostringstream os;
        auto sc_now = std::chrono::system_clock::now();
        auto tm_now = std::chrono::system_clock::to_time_t(sc_now);
        fmt::print("{:%Y-%m-%d}\n", *std::localtime(&tm_now));
        os << "date : " << fmt::format("{:%Y-%m-%d}\n", *std::localtime(&tm_now)); 
        auto tm_now2  = std::time(nullptr);
        fmt::print("{:%H:%I:%S}\n", *std::localtime(&tm_now2));
        os << " time : " << fmt::format("{:%H:%I:%S}\n", *std::localtime(&tm_now2));
        return crow::response{os.str()};
   });

   //      * curl -d '{"pad":123,"by":6'} {ip}:40081/pad_json  
   CROW_ROUTE(app, "/pad_json")
        .methods(crow::HTTPMethod::POST)
   ([&](const crow::request& req){
        auto x = crow::json::load(req.body);
        if (!x)
            return crow::response(400);
        auto tag = x["pad"].i();
		auto by_int = x["by"].i();
		fmt::print("{:0{}d}\n", tag, by_int);
        std::string fmt_tag = fmt::format("{:0{}d}\n", tag, by_int);
        return crow::response{fmt_tag};
   });
   
  app.port(40081)
    //.multithreaded()
    .run();
}