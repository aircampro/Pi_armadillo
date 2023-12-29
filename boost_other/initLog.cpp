void InitializeFileLog(const std::String & logDir)
	    {   
		            boost::shared_ptr< logging::core > loggingCore = logging::core::get();


			            loggingCore->add_global_attribute("TimeStamp", attrs::local_clock());

				            string logPath = logDir + "/gvzmrcpsr_%N.txt";

					            boost::shared_ptr< sinks::text_file_backend > backend =
							                boost::make_shared< sinks::text_file_backend >(
											                // file name pattern
													//                 keywords::file_name = logPath,
													//                                 // rotate the file upon reaching 5 MiB size...
													//                                                 keywords::rotation_size = 5 * 1024 * 1024,
													//                                                                 // ...or at midnight, whichever comes first
													//                                                                                 keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0)                    
													//                                                                                             );
													//
													//                                                                                                     backend->auto_flush(true);
													//
													//                                                                                                             // Wrap it into the frontend and register in the core.
													//                                                                                                                     // The backend requires synchronization in the frontend.
													//                                                                                                                             typedef sinks::synchronous_sink< sinks::text_file_backend > sink_t;
													//                                                                                                                                     boost::shared_ptr< sink_t > sink = boost::make_shared< sink_t>(backend);
													//
													//                                                                                                                                             loggingCore->add_sink(sink);
													//
													//
													//                                                                                                                                                     sink->flush();
													//                                                                                                                                                             sink->set_formatter
													//                                                                                                                                                                         (
													//                                                                                                                                                                                     expr::stream
													//                                                                                                                                                                                                 << expr::attr< boost::posix_time::ptime >("TimeStamp")
													//                                                                                                                                                                                                             << " : [" << expr::attr< sestek::log::LogLevel >("Severity")
													//                                                                                                                                                                                                                         << "] " << expr::smessage
													//                                                                                                                                                                                                                                     );
													//
													//                                                                                                                                                                                                                                             backend->set_file_collector(sinks::file::make_collector(
													//                                                                                                                                                                                                                                                         // rotated logs will be moved here
													//                                                                                                                                                                                                                                                                     keywords::target = logDir + "/old_mrcpsr_plugin_logs",
													//                                                                                                                                                                                                                                                                                 // oldest log files will be removed if the total size reaches 100 MiB...
													//                                                                                                                                                                                                                                                                                             keywords::max_size = 100 * 1024 * 1024,
													//                                                                                                                                                                                                                                                                                                         // ...or the free space in the target directory comes down to 50 MiB
													//                                                                                                                                                                                                                                                                                                                     keywords::min_free_space = 50 * 1024 * 1024
													//                                                                                                                                                                                                                                                                                                                             ));
													//
													//                                                                                                                                                                                                                                                                                                                                     try
													//                                                                                                                                                                                                                                                                                                                                             {
													//                                                                                                                                                                                                                                                                                                                                                         backend->scan_for_files(sinks::file::scan_all);
													//                                                                                                                                                                                                                                                                                                                                                                 }
													//                                                                                                                                                                                                                                                                                                                                                                         catch(std::exception & )
													//                                                                                                                                                                                                                                                                                                                                                                                 {
													//                                                                                                                                                                                                                                                                                                                                                                                             //LOGGER(sestek::log::fatal) << "exception during scanning : " << e.what();
													//
													//                                                                                                                                                                                                                                                                                                                                                                                                     }
													//
													//                                                                                                                                                                                                                                                                                                                                                                                                         }
