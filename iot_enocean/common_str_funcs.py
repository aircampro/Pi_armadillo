#
# common string function library with html parsers
# common http client request to server for url
# timestamp generator
#
from bs4 import BeautifulSoup
from tika import parser
import datetime
import pytz
import urllib.request
import re
import math
 
def remove_string( str_in, rem_str ):
    return str_in.replace(rem_str, '')

def remove_string_ntimes( str_in, rem_str, n ):
    return str_in.replace(rem_str, '', n)
    
def replace_char_or_string( str_in, rem_char, rep_char ):
    return str_in.replace(rem_char, rep_char)

def replace_char_or_string_ntimes( str_in, rem_char, rep_char, n ):
    return str_in.replace(rem_char, rep_char, n)

def remove_tab_newlines( str_in ):
    cleaned_dt_list = ""
    for dt in str_in:
        dt = ''.join(str(dt).split('\t', 1))
        dt = ''.join(str(dt).split('\n', 1))
        cleaned_dt_list += dt
    return cleaned_dt_list		

def remove_char( str_in, chara ):
    cleaned_dt_list = ""
    for dt in str_in:
        dt = ''.join(str(dt).split(chara, 1))
        cleaned_dt_list += dt
    return cleaned_dt_list

def remove_char_ntimes( str_in, chara, n ):
    cleaned_dt_list = ""
    c = 0
    for dt in str_in:
        if len(str(dt).split(chara, 1)) == 2:
            c += 1
        if c < (n+1) :
            dt = ''.join(str(dt).split(chara, 1))
        cleaned_dt_list += dt
    return cleaned_dt_list
    
# Function for cleaning raw text
def cleaning_raw_text(text_strings):
    safe_text = text_strings.encode('utf-8', errors='ignore')
    safe_text = safe_text.decode('utf-8')
    clean_text = str(safe_text).replace("\nn", "\n")
    clean_text = str(clean_text).replace("\nnn", "\n")
    clean_text = str(clean_text).replace("\n\n\n\n\n", "\n")
    clean_text = str(clean_text).replace("\n\n\n\n", "\n")
    clean_text = str(clean_text).replace("\n\n\n", "\n")
    clean_text = str(clean_text).replace("\n\n", "\n")
    clean_text = str(clean_text).replace("\n\n", "\n")
    clean_text = str(clean_text).replace("-----", "-")
    clean_text = str(clean_text).replace("----", "-")
    clean_text = str(clean_text).replace("---", "-")
    clean_text = ''.join(clean_text.split('\n', 1))
    return clean_text

def create_url_list(html_text_list):
    MAX_URL_SIZE = 50
    SITES = ('https://www.google.com/', 'https://www.facebook.com')
    all_urls_list = []
    for text_with_urls in html_text_list:
        urls = re.findall(r"""(?i)\b((?:https?:(?:/{1,3}|[a-z0-9%])|[a-z0-9.\-]+[.](?:com|net|org|edu|gov|mil|aero|asia|biz|cat|coop|info|int|jobs|mobi|museum|name|post|pro|tel|travel|xxx|ac|ad|ae|af|ag|ai|al|am|an|ao|aq|ar|as|at|au|aw|ax|az|ba|bb|bd|be|bf|bg|bh|bi|bj|bm|bn|bo|br|bs|bt|bv|bw|by|bz|ca|cc|cd|cf|cg|ch|ci|ck|cl|cm|cn|co|cr|cs|cu|cv|cx|cy|cz|dd|de|dj|dk|dm|do|dz|ec|ee|eg|eh|er|es|et|eu|fi|fj|fk|fm|fo|fr|ga|gb|gd|ge|gf|gg|gh|gi|gl|gm|gn|gp|gq|gr|gs|gt|gu|gw|gy|hk|hm|hn|hr|ht|hu|id|ie|il|im|in|io|iq|ir|is|it|je|jm|jo|jp|ke|kg|kh|ki|km|kn|kp|kr|kw|ky|kz|la|lb|lc|li|lk|lr|ls|lt|lu|lv|ly|ma|mc|md|me|mg|mh|mk|ml|mm|mn|mo|mp|mq|mr|ms|mt|mu|mv|mw|mx|my|mz|na|nc|ne|nf|ng|ni|nl|no|np|nr|nu|nz|om|pa|pe|pf|pg|ph|pk|pl|pm|pn|pr|ps|pt|pw|py|qa|re|ro|rs|ru|rw|sa|sb|sc|sd|se|sg|sh|si|sj|Ja|sk|sl|sm|sn|so|sr|ss|st|su|sv|sx|sy|sz|tc|td|tf|tg|th|tj|tk|tl|tm|tn|to|tp|tr|tt|tv|tw|tz|ua|ug|uk|us|uy|uz|va|vc|ve|vg|vi|vn|vu|wf|ws|ye|yt|yu|za|zm|zw)/)(?:[^\s()<>{}\[\]]+|\([^\s()]*?\([^\s()]+\)[^\s()]*?\)|\([^\s]+?\))+(?:\([^\s()]*?\([^\s()]+\)[^\s()]*?\)|\([^\s]+?\)|[^\s`!()\[\]{};:'\".,<>?«»“”‘’])|(?:(?<!@)[a-z0-9]+(?:[.\-][a-z0-9]+)*[.](?:com|net|org|edu|gov|mil|aero|asia|biz|cat|coop|info|int|jobs|mobi|museum|name|post|pro|tel|travel|xxx|ac|ad|ae|af|ag|ai|al|am|an|ao|aq|ar|as|at|au|aw|ax|az|ba|bb|bd|be|bf|bg|bh|bi|bj|bm|bn|bo|br|bs|bt|bv|bw|by|bz|ca|cc|cd|cf|cg|ch|ci|ck|cl|cm|cn|co|cr|cs|cu|cv|cx|cy|cz|dd|de|dj|dk|dm|do|dz|ec|ee|eg|eh|er|es|et|eu|fi|fj|fk|fm|fo|fr|ga|gb|gd|ge|gf|gg|gh|gi|gl|gm|gn|gp|gq|gr|gs|gt|gu|gw|gy|hk|hm|hn|hr|ht|hu|id|ie|il|im|in|io|iq|ir|is|it|je|jm|jo|jp|ke|kg|kh|ki|km|kn|kp|kr|kw|ky|kz|la|lb|lc|li|lk|lr|ls|lt|lu|lv|ly|ma|mc|md|me|mg|mh|mk|ml|mm|mn|mo|mp|mq|mr|ms|mt|mu|mv|mw|mx|my|mz|na|nc|ne|nf|ng|ni|nl|no|np|nr|nu|nz|om|pa|pe|pf|pg|ph|pk|pl|pm|pn|pr|ps|pt|pw|py|qa|re|ro|rs|ru|rw|sa|sb|sc|sd|se|sg|sh|si|sj|Ja|sk|sl|sm|sn|so|sr|ss|st|su|sv|sx|sy|sz|tc|td|tf|tg|th|tj|tk|tl|tm|tn|to|tp|tr|tt|tv|tw|tz|ua|ug|uk|us|uy|uz|va|vc|ve|vg|vi|vn|vu|wf|ws|ye|yt|yu|za|zm|zw)\b/?(?!@)))""", text_with_urls)
        url_list = []
        sites_urls_list = []
        for url in urls:
            url = re.sub('\?source.+', '', url)
            if url.startswith(SITES) and len(url) > MAX_URL_SIZE:
                sites_urls_list.append(url)
            else:
                url_list.append(url)
        all_urls_list.append(url_list)
        all_urls_list.append(sites_urls_list)
    return all_urls_list 
		
def read_html(filename):
    file = parser.from_file(filename)
    return (file)
	
def html_to_txt(html):
    soup = BeautifulSoup(html, 'lxml')
    # removing scripts, styles and other useless tags
    [element.extract() for element in soup(['style', 'script', 'meta', '[document]', 'head', 'title'])]
    # getting text from html
    text = soup.getText()
    return text

def html_images(html):
    soup = BeautifulSoup(html, 'lxml')
    articles = soup.find_all('div', class_='post')
    url_list = []
    for article in articles:
        images = article.find_all('img')
        for img in images:
            img_url = img['src']
            print(f"Image URL: {img_url}")
            url_list.append(img_url) 
    return url_list            
   
# URL request function
def url_request(url):
    # http://www.networkinghowtos.com/howto/common-user-agent-list/
    HEADERS = ({'User-Agent':
            'Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:53.0) Gecko/20100101 Firefox/53.0',
            'Accept-Language': 'uk-UA, ukr;q=0.5'})
    request = urllib.request.Request(url, headers=HEADERS)
    response = urllib.request.urlopen(request, timeout=5)
    status_code = response.status
    print(f'Status code {status_code}')

    # if no error, then read the response contents
    if 200 <= status_code < 300:
    # read the data from the URL
        data_response = response.read().decode("utf8")
    return data_response

def get_url_with_soup(ur='http://example.com'):
    response = url_request(url)(ur)
    soup = BeautifulSoup(response, 'html.parser')  
    return soup
    
def get_all_links(soup_v):
    all_links = soup_v.find_all('a')
    arr = []
    for link in all_links:
        print(link.get('href'))
        arr.append(link.get('href'))
    return arr

def get_all_img(soup_v):
    articles = soup_v.find_all('div', class_='post')
    url_list = []
    for article in articles:
        images = article.find_all('img')
        for img in images:
            img_url = img['src']
            print(f"Image URL: {img_url}")
            url_list.append(img_url) 
    return url_list 
    
def create_timestamp():
    # This timestamp is in UTC
    my_ct = datetime.datetime.now(tz=pytz.UTC)
    tz = pytz.timezone('Europe/London')
    # Now convert it to another timezone
    new_ct = my_ct.astimezone(tz)
    timestamp = new_ct.strftime("%d-%m-%Y_%H-%I")
    return timestamp
    
# （）,(),『』,「」,〝〟,“”,"",'',``
# 512 is the threshold for skipping over a forgotten closing parenthesis
SPEECH_BLOCK_PATTERNS = (
    r"（[^（）]{0,512}）", r"\([^\(\)]{0,512}\)",
    r"『[^『』]{0,512}』", r"「[^「」]{0,512}」",
    r"〝[^〝〟]{0,512}〟", r"“[^“”]{0,512}”",
    r'"[^"]{0,512}"', r"'[^']{0,512}'", r"`[^`]{0,512}`"
)
SPEECH_BRACKETS = set("（）()『』「」〝〟“”""'`")

def separate_speech_lines(text):
    if isinstance(text, (list, tuple)):
        text = "\n".join(text)
    speech_blocks = sum([[m[0] for m in re.finditer(regex, text, re.M)]
                         for regex in SPEECH_BLOCK_PATTERNS], [])

    non_speech_text = text
    for block in sorted(speech_blocks, key=lambda line: len(line), reverse=True):
        non_speech_text = non_speech_text.replace(block, "")
    non_speech_text = re.sub(r"[\r\n]+", "\n", non_speech_text)
    non_speech_blocks = [block for block in non_speech_text.split("\n")
                         if all(b not in block for b in SPEECH_BRACKETS)]

    speech_lines = sum([split_sentence(block[1:-1].strip(" \r\n\t　"))
                        for block in speech_blocks], [])
    non_speech_lines = sum([split_sentence(block.strip(" \t　"))
                            for block in non_speech_blocks], [])

    return speech_lines, non_speech_lines

def split_sentence(text):
    text = re.sub(r"([。\.\?？！]+)", r"\1\n", text)
    text = re.sub(r"[\r\n]+", "\n", text)
    lines = remove_empty([line.strip(" 　\t") for line in text.split("\n")])
    return lines

def remove_punct(line):
    return re.sub(r"[。、\.,\?？！]+", "", line)

def remove_empty(lines):
    return [line for line in lines if line.strip(" 　\t\r\n")]

def filter_length(lines, min_len=0, max_len=math.inf):
    return [line for line in lines if min_len <= len(line) <= max_len]
	
def load_content(text_file):
    content = []
    with open(text_file, mode="r", encoding="cp932") as f:
        lines = list(f.readlines())
        start = False
        for i, line in enumerate(lines):
            if not start:
                if line.startswith("----") and i + 1 < len(lines) and lines[i + 1] == "\n":
                    start = True
                continue

            line = re.sub(r"^[\s　]+", "", line.strip())
            if not line:
                continue
            if line[0] in {"＊", "＃", "［"}:
                continue
            if all([c == "―" for c in line]):
                continue

            line = re.sub(r"※［＃.*］", "", line)
            line = re.sub(r"＊[１２３４５６７８９０]*［＃.*］", "", line)
            line = re.sub(r"［＃.*］", "", line)
            line = re.sub(r"＊[１２３４５６７８９０　]*", "", line)
            line = re.sub(r"《.*》", "", line)
            line = re.sub(r"｜", "", line)
            if line.startswith("底本："):
                break
            if line:
                content.append(line)
    return "\n".join(content)

def load_resource(text_file):
    content = load_content(text_file)
    return separate_speech_lines(content)

def load_speech_lines(text_file, remove_punct=False, min_len=3, max_len=math.inf):
    speech_lines, non_speech_lines = load_resource(text_file)
    if remove_punct:
        speech_lines = remove_empty([remove_punct(line) for line in speech_lines])

    return filter_length(speech_lines, min_len=min_len, max_len=max_len)

def load_non_speech_lines(text_file, remove_punct=False, min_len=3, max_len=math.inf):
    speech_lines, non_speech_lines = load_resource(text_file)
    if remove_punct:
        non_speech_lines = remove_empty([remove_punct(line) for line in non_speech_lines])

    return filter_length(non_speech_lines, min_len=min_len, max_len=max_len)

	
