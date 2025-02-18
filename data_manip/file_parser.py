#!/usr/bin/env python
#
# Example of simple file parser for chosen code page
#

# remove empty lines
def remove_empty(lines):
    return [line for line in lines if line.strip(" 　\t\r\n")]

# filter line on length
def filter_length(lines, min_len=0, max_len=math.inf):
    return [line for line in lines if min_len <= len(line) <= max_len]
	
# a simple file parser with some simple rules for data manipulation
#
def load_content(text_file, enco="utf-8"):
    content = []
    with open(text_file, mode="r", encoding=enco) as f:
        lines = list(f.readlines())
        start = False
        for i, line in enumerate(lines):
            if not start:
                if line.startswith("----") and i + 1 < len(lines) and lines[i + 1] == "\n":            # detect start of the lines
                    start = True
                continue

            line = re.sub(r"^[\s　]+", "", line.strip())
            if not line:
                continue
            if line[0] in {"*", "#", "["}:
                continue
            if all([c == "―" for c in line]):
                continue

            # add any supported code pages here (we are currently only japanese and utf-8)
            if enco == "cp932":                                                                        # japanese encoding was chosen code page 932
			    line = re.sub(r"＃.*", "", line)
                line = re.sub(r"[１２３４５６７８９０]", "", line)
				line = re.sub(r"｜.*", "", line)
                skip_line = "底本："
			elif enco == "utf-8":                                                                      # file is standard utf-8 (e.g. ascii with extended chars like emoji)
                line = re.sub(r"#.*", "", line)                                                        # remove # comment
			    line = re.sub(r"[1234567890]", "", line)                                               # remove numbers
                line = re.sub(r"\|.*", "", line)                                                       # remove | comment
			    line = re.sub(r":", ">", line)                                                         # change : to >
			    re.sub(r"[\.,\?]+", "", line)                                                          # remove punctuation
				skip_line = "step:"
            if line.startswith(skip_line):                                                             # skip this line
                break
            if line:
                content.append(line)
    return "\n".join(content)