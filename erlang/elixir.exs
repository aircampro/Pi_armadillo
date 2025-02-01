# This is simple elixir code to do some functions
# it also shows how to embed some erlang functions and libs 
#
name = "Elixir"

use Bitwise

# send a fixed message via a pipe
defmodule A do
  def greet1(), do: "Hello, World!" |> IO.puts()
end

# say name via a call to a erlang module (say)
defmodule B do
  def greet2(name) do
    say(name)
  end
  defp say(name), do: :io.format("hello ~s~n",name)
end

# same but no erlang function and check the age passed through the function
defmodule C do
  def hello(name) do
    IO.puts "Hello, #{name}!"
  end
  def only_when({age: age}) when age >= 16 do
    IO.puts "Age is, #{age}!"
  end
end

# do factorial
defmodule Factorial do
  def of(0), do: 1
  def of(n), do: n * of(n - 1)
end

# split sentance
defmodule SplitSentance do
  def split_by_space(str) do
    String.split(str, " ")
  end
  def split_by_colon(str) do
    String.split(str, ":")
  end
end

# spawn and send messaage
defmodule spawn_send do
  def send_rcv(msg_send) do
    current_process = self()
    # Spawn an Elixir process (not an operating system one!)
    spawn_link(fn ->  send current_process, {:msg, msg_send}end)
    # Block until the message is received
    receive do {:msg, contents} -> IO.puts contents end
  end
end

# file operation
defmodule file_ops do
  def write_txt(filen, txt) do
    {:ok, file} = File.open(filen, [:write])
    IO.binwrite(file, txt)
    File.close(file)
  end
  def read_file(filen) do
    {:ok, content} = File.read(filen)
    IO.puts("file read: #{content}")
  end
end

# load erland libs for http
defp deps do
  [
    {:httpoison, "~> 1.8"},
    {:floki, "~> 0.30.0"}
  ]
end

# request url
defmodule http_req do
  def fetch_titles(url) do
    {:ok, response} = HTTPoison.get(url)
    {:ok, document} = Floki.parse_document(response.body)
    titles = Floki.find(document, "h1") |> Floki.text()
    IO.puts("titles from url : #{titles}")
  end
end

# load erland lib for json
defp deps do
  [
    {:jason, "~> 1.2"}
  ]
end

defmodule JsonExample do
  def encode_and_decode do
    data = %{name: "jay sun", age: 30}
    json = Jason.encode!(data)
    IO.puts("JSON: #{json}")

    decoded = Jason.decode!(json)
    IO.inspect(decoded)
  end
end

# 2 threads
defmodule ConcurrentTasks do
  def run_tasks do
    task1 = Task.async(fn -> perform_task("task_1") end)
    task2 = Task.async(fn -> perform_task("task_2") end)

    Task.await(task1)
    Task.await(task2)
  end

  defp perform_task(name) do
    IO.puts("#{name} is running...")
    :timer.sleep(1000)
    IO.puts("#{name} is running")
  end
end

# crypto
defmodule crypt do
  def cryp(cc, mode) do
    cond do
      mode == 1 ->
        crypto.hash(:md5, cc)
      mode == 2 ->
        crypto.hash(:sha, cc)
      mode == 3 ->
        crypto.hash(:sha224, cc)
      mode == 4 ->
        crypto.hash(:sha256, cc)
      mode == 5 ->
        crypto.hash(:sha384, cc)
      mode == 6 ->
        crypto.hash(:sha512, cc)
      true -> 
        "invalid mode md5=1 sha1=2 sha224=3 sha256=4 sha384=5 sha512=6"
    end
  end
  def hmac(keykey, mesg) do
    crypto.hmac(:sha256, keykey, mesg) |> Base.encode16(case: :lower) 
  end
end

defmodule Digest do
  def sha256(data) do
    :crypto.hash(:sha256, data) |> Base.encode16(case: :lower)
  end
  # CTR mode has the nature of a stream cipher, so there is no need to worry about padding.
  #
  # omit creation of key/iv
  # encryption
  def ctr_aes_cipher(text2cipher) do
    key = :crypto.strong_rand_bytes(16)
    iv  = :crypto.strong_rand_bytes(16)
    state = :crypto.stream_init(:aes_ctr, key, iv)
    {new_state, cipher} = :crypto.stream_encrypt(state, text2cipher)
    IO.puts cipher
    # decryption
    state = :crypto.stream_init(:aes_ctr, key, iv)
    {new_state, plaintext} = :crypto.stream_decrypt(state, cipher)
    IO.puts plaintext
  end

  # AES-GCM Mode
  # Authentication example
  #
  def gcm_aes_cipher(text2cipher1, text2cipher2) do
    key = :crypto.strong_rand_bytes(16)
    iv  = :crypto.strong_rand_bytes(16)
    {cipher, ctag} = :crypto.block_encrypt(:aes_gcm, key, iv, {"aad", text2cipher2})
    IO.puts cipher
    # this successfully decrypts and returns plaintext
    {cleantext, ctag} = :crypto.block_decrypt(:aes_gcm, key, iv, {"aad", cipher, ctag})
    IO.puts cleantext
  end
end

defmodule Crypto do
    def md5(s) do
        list_to_binary(Enum.map(bitstring_to_list(:crypto.md5(s)), fn(x) -> integer_to_binary(x, 16) end))
    end
end

# case statement
defmodule CaseExample do
   def exam_case(weather) do
    case weather do
     "sunny" -> "go outside"
     "dry" -> "go outside"
     "hot" -> "go outside"
     "rainy" -> "read books"
     "rain" -> "read books"
     "wet" -> "read books"
     "cloudy" -> "tidy up"
     _ -> "not listed! do some other things"
  end  
 end
end

# chatbot LiveView (lib/my_phoenix_app_web/live/chat_room_live.ex)
defmodule MyPhoenixAppWeb.ChatRoomLive do
  use Phoenix.LiveView

  def mount(_params, _session, socket) do
    {:ok, assign(socket, :messages, [])}
  end

  def handle_event("send_message", %{"message" => message}, socket) do
    {:noreply, update(socket, :messages, &[message | &1])}
  end

  def render(assigns) do
    ~L"""
    <div>
      <h1>ChatRoom</h1>
      <ul>
        <%= for message <- @messages do %>
          <li><%= message %></li>
        <% end %>
      </ul>
      <form phx-submit="send_message">
        <input name="message" type="text" placeholder="message" />
        <button type="submit">send</button>
      </form>
    </div>
    """
  end
end

# LiveView using Phoenix Web App (lib/my_phoenix_app_web/live/counter_live.ex)
defmodule MyPhoenixAppWeb.CounterLive do
  use Phoenix.LiveView

  def mount(_params, _session, socket) do
    {:ok, assign(socket, :count, 0)}
  end

  def handle_event("increment", _value, socket) do
    {:noreply, update(socket, :count, &(&1 + 1))}
  end

  def render(assigns) do
    ~L"""
    <div>
      <h1>Your Counter is: <%= @count %></h1>
      <button phx-click="increment">counter add</button>
    </div>
    """
  end
end

# index page (lib/my_phoenix_app_web/controllers/page_controller.ex)
defmodule MyPhoenixAppWeb.PageController do
  use MyPhoenixAppWeb, :controller

  def index(conn, _params) do
    render(conn, "index.html")
  end
end

# run the functions
#
A.greet1()
B.greet2("peter")
C.hello("john")
C.only_when(10)
C.only_when(17)
C.only_when(15)
C.only_when(16)
Factorial.of(32)
Factorial.of(0)
SplitSentance.split_by_space("hello world")
SplitSentance.split_by_colon("12:20:10")
spawn_send.send_rcv("send a message from thread to thread")
crypt.cryp("Using crypto from Erlang OTP",1)
crypt.hmac("your_key", "Using crypto from Erlang OTP")
Digest.sha256("message to encode")
Digest.ctr_aes_cipher("yes this is it 1234")
Digest.gcm_aes_cipher("yes this is it 1234")
Crypto.md5("list of bianry from this msg")
ConcurrentTasks.run_tasks()
JsonExample.encode_and_decode()
http_req.fetch_titles("https://militants-of-funk.tripod.com")
file_ops.read_file("myfile.txt")
file_ops.write_txt("myfile.txt", "writen this to the file")
file_ops.read_file("myfile.txt")
CaseExample.exam_case("rainy")

# run phoenix web apps
pipeline :api do
  plug :accepts, ["json"]
end

scope "/", HelloWeb do
  pipe_through :browser

  live "/counter", CounterLive
  live "/chat", ChatRoomLive
  get "/", PageController, :index
end
