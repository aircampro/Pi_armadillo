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

def create_timestamp():
    # This timestamp is in UTC
    my_ct = datetime.datetime.now(tz=pytz.UTC)
    tz = pytz.timezone('Europe/London')
    # Now convert it to another timezone
    new_ct = my_ct.astimezone(tz)
    timestamp = new_ct.strftime("%d-%m-%Y_%H-%I")
    return timestamp