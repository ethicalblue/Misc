/*
 * C++ implementation of NRV2E decompression algorithm
 * which was used in this project to decode
 * AZTEC 2D from vehicle registration documents
 * - by ethical.blue // 2019
 *
 * Based on original UCL library written by:
 * Markus F.X.J. Oberhumer <markus@oberhumer.com>
 * http://www.oberhumer.com/opensource/ucl/
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <fstream>
#include <vector>

const signed START_OFFSET{ 4 };
signed ilen{ START_OFFSET };
unsigned current_bit{ 0 };
std::byte current_byte{ 0 };
std::vector<std::byte> src{ 0 };

static unsigned get_bit()
{
	if (ilen >= src.size())
		throw std::invalid_argument("Przesunięcie jest poza zakresem.");

	if (current_bit == 0)
	{
		current_byte = src[ilen++];
		current_bit = 8;
	}

	return (static_cast<unsigned>(current_byte) >> --current_bit) & 1;
}

static std::vector<std::byte> decompress_NRV2E(const std::vector<std::byte>& source_data)
{
	src = source_data;

	unsigned dest_size{ static_cast<unsigned>(src[0]) bitor
						 static_cast<unsigned>(src[1]) << 8u bitor
						 static_cast<unsigned>(src[2]) << 16u bitor
						 static_cast<unsigned>(src[3]) << 24u };

	std::vector<std::byte> dst(dest_size);

	unsigned olen{ 0 };
	unsigned last_m_off{ 1 };

	while (ilen < src.size())
	{
		unsigned m_off{ 0 };
		unsigned m_len{ 0 };

		while (get_bit() == 1)
		{
			dst[olen++] = src[ilen++];
		}

		m_off = 1;
		while (true)
		{
			m_off = m_off * 2 + get_bit();
			if (get_bit() == 1)
				break;
			m_off = (m_off - 1) * 2 + get_bit();
		}

		if (m_off == 2)
		{
			m_off = last_m_off;
			m_len = get_bit();
		}
		else
		{
			m_off = (m_off - 3) * 256 + static_cast<unsigned>(src[ilen++]);
			if (m_off == std::numeric_limits<unsigned>().max())
				break;
			m_len = (m_off ^ std::numeric_limits<unsigned>().max()) & 1;
			m_off >>= 1;
			last_m_off = ++m_off;
		}
		if (m_len > 0)
			m_len = 1 + get_bit();
		else if (get_bit() == 1)
			m_len = 3 + get_bit();
		else
		{
			m_len++;
			do
			{
				m_len = m_len * 2 + get_bit();
			} while (get_bit() == 0);
			m_len += 3;
		}
		m_len += m_off > 0x500 ? 1 : 0;

		unsigned m_pos{ 0 };
		m_pos = olen - m_off;

		dst[olen++] = dst[m_pos++];
		do dst[olen++] = dst[m_pos++]; while (--m_len > 0);
	}
	return dst;
}

static std::vector<std::byte> base64_decode(const std::vector<std::byte>& in) {

	std::vector<std::byte> out{ 0 };

	std::vector<int> T(256, -1);
	for (int i = 0; i < 64; i++)
		T["ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i]] = i;

	unsigned val{ 0 };
	signed valb{ -8 };
	for (auto b : in)
	{
		if (T[static_cast<unsigned>(b)] == -1) break;
		val = (val << 6) + T[static_cast<unsigned>(b)];
		valb += 6;
		if (valb >= 0)
		{
			out.push_back(static_cast<std::byte>((val >> valb) & 0xFF));
			valb -= 8;
		}
	}
	return out;
}

int main()
{
	std::string test1{ "BgQAANtYAAJDAPkxAHwAQXIw7zcGNN4ANiox+w81HrUGOP8eUABSAEUA+1oAWQBEDv9OAFQAIABN3wAuClMAvlQPV/eKUhq9Wg5X7k58UtcWSVq9TF5J79pBZ+5PAEsG12bTSm5GVQBM/ntSAEH7L1dj+0MAS1vvMvovewo3Ut4wDi39HjEAN6Pbl0FNe3YgPt5Q3kv3IlSevVnX1z9FMmuCShL2WgBaG9umKADvSAApJnx75k+itwZMAEx9X0rvbkSOTXtOOF/DRy0WOW53fPYLFoMzLr0xAi3DGnevLQOCfJ/vQZ5TcBZrN0oa9k4AfA82Q4QaDzj3q8deN6sN7zIE/1x8lbMnQdwBQi5ZT86jL2tqNAr2MwAw34xSH+uPSVPYFxZThBMzON8AMJM5wQA3MwRcMX7bNcET2jInwyedE01HZ4dlM94qKy0DL38fNgAqeBszSxOvNIeKfHM7fCLxNQAwVkMtdzl7Xiw/YMyrFzxQACBWw+Hza7c3C93/NWuHg1OWRquPQ5KP02K9IBZT4QZC9oNZU7aXFiOX83U4ADJFC7ADhrNVCyOW8w9qMbEnZhdHbHxjdjIT7E4DW0M3OQuGaxYmCSSSSSr/" };

	if (test1.length() % 2 == 1)
		test1[test1.length() - 1] = u8'\0';

	std::vector<std::byte> text_bytes{ 0 };

	for (char c : test1)
		text_bytes.push_back(static_cast<std::byte>(c));

	std::vector<std::byte> decoded = base64_decode(text_bytes);
	std::vector<std::byte> decompressed = decompress_NRV2E(decoded);
	std::u16string plain_data{ 0 };
	plain_data.assign(reinterpret_cast<std::u16string::const_pointer>(&decompressed[0]), decompressed.size() / sizeof(std::u16string::value_type));

	/* Zapisz rozkodowane dane do pliku tekstowego
	(zmień ścieżkę według swojego systemu) */
	std::basic_ofstream<char16_t> outfile("C:\\Users\\x\\Desktop\\Nowy folder\\file1.txt", std::ios_base::binary);
	outfile.write(plain_data.c_str(), plain_data.length());
	outfile.close();

	std::string visit{ "https://ethical.blue" };

	return EXIT_SUCCESS;
}