#include <SerialIO.h>

// ***  SINGLETON  *** //
// ******************* //
SerialIO* SerialIO::instance = nullptr;
SerialIO*
SerialIO::getInstance()
{
  if (instance == nullptr)
    instance = new SerialIO();
  return instance;
}

// ***  INNER UTIL METHODS  *** //
// **************************** //
void
SerialIO::handleArchiveException(std::string const& perpetrator,
                                 boost::archive::archive_exception& aex)
{
  std::stringstream ss;
  ss << perpetrator << " ARCHIVE EXCEPTION:\n\t" << aex.what() << "\n\t"
     << "CODE: ";
  switch (aex.code) {
    case archive_exception::no_exception:
      ss << "no_exception";
      break;
    case archive_exception::other_exception:
      ss << "other_exception";
      break;
    case archive_exception::unregistered_class:
      ss << "unregistered_class\n\t"
         << "Attempt to serialize a pointer of an unregistered "
         << "class";
      break;
    case archive_exception::invalid_signature:
      ss << "invalid_signature\n\t"
         << "First line of archive does not contain expected "
         << "string";
      break;
    case archive_exception::unsupported_version:
      ss << "unsupported_version\n\t"
         << "Archive created with library version subsequent to "
         << "this one";
      break;
    case archive_exception::pointer_conflict:
      ss << "pointer_conflict\n\t"
         << "An attempt has been made to directly serialize an "
         << "object after having already serialized the same "
         << "object through a pointer.\n\t"
         << "Were this permitted, the archive load would result in "
         << "the creation of an extraneous object.";
      break;
    case archive_exception::incompatible_native_format:
      ss << "incompatible_native_format\n\t"
         << "Attempt to read native binary format on incompatible "
         << "platform";
      break;
    case archive_exception::array_size_too_short:
      ss << "array_size_too_short\n\t"
         << "Array being loaded doesnt fit in array allocated";
      break;
    case archive_exception::input_stream_error:
      ss << "input_stream_error\n\t"
         << "Error on stream input.\n\t"
         << "Maybe uninitialized data has been passed to "
            "serialization module.\n\t"
         << "Another option would be read class type is not the same "
            "as written class type";
      break;
    case archive_exception::invalid_class_name:
      ss << "invalid_class_name\n\t"
         << "Class name greater than the maximum permitted.\n\t"
         << "Most likely a corrupted archive or an attempt to "
         << "insert virus via buffer overrun method";
      break;
    case archive_exception::unregistered_cast:
      ss << "unregistered_cast\n\t"
         << "Base - derived relationship not registered with "
         << "void_cast_register";
      break;
    case archive_exception::unsupported_class_version:
      ss << "unsupported_class_version\n\t"
         << "Type saved with a version greater than the one used "
         << "by the program.\n\t"
         << "This indicates that the program needs to be rebuilt";
      break;
    case archive_exception::multiple_code_instantiation:
      ss << "multiple_code_instantiation\n\t"
         << "Code for implementing serialization for some type has "
         << "been instantiated in more than one module";
      break;
    case archive_exception::output_stream_error:
      ss << "output_stream_error\n\t"
         << "Error on stream output";
      break;
  }
  logging::WARN(ss.str());
}
