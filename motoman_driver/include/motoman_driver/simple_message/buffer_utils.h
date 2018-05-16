#pragma once
inline bool loadString(industrial::byte_array::ByteArray& buffer, const std::string& value, size_t field_length)
{
  std::vector<char> buf(field_length, 0);
  memcpy(buf.data(), value.c_str(), value.length() + 1);
  industrial::byte_array::ByteArray tmp_buffer;
  tmp_buffer.init(buf.data(), field_length);
  return buffer.load(tmp_buffer);
}

inline bool unloadString(industrial::byte_array::ByteArray& buffer, std::string& value, size_t field_length)
{
  std::vector<char> buf(field_length, 0);
  bool res = buffer.unload(buf.data(), field_length);
  value = buf.data();
  return res;
}

inline bool loadByteArray(industrial::byte_array::ByteArray& buffer, const std::vector<char>& value, size_t field_length)
{
  std::vector<char> tmp_vector = value;
  tmp_vector.resize(field_length, 0);
  industrial::byte_array::ByteArray tmp_buffer;
  tmp_buffer.init( tmp_vector.data(), tmp_vector.size() );
  return buffer.load(tmp_buffer);
}

inline bool unloadByteArray(industrial::byte_array::ByteArray& buffer, std::vector<char>& value, size_t field_length)
{
  value.resize(field_length, 0);
  return buffer.unload( value.data(), value.size() );
}