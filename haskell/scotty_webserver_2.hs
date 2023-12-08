-- Haskell webserver using scotty - this is a veraion for another version of compiler
-- add users to the system and query them from the uid number 

-- add to package yaml
-- http-types
-- aeson
-- scotty

-- stack ghc scotty_webserver.hs
-- stack runghc scotty_webserver.hs

{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE DeriveGeneric #-}

module UserAPI where

import Web.Scotty
import Data.Aeson (FromJSON, ToJSON)
import GHC.Generics
import Data.IORef
import Control.Monad.Reader
import Network.HTTP.Types.Status
import Data.List (find)
import System.Environment (lookupEnv)
import System.IO
import qualified Data.Text as T
import qualified Data.Text.Encoding as T
import qualified Codec.Text.IConv as I
import qualified Data.ByteString.Lazy as L

-- the users of this server have a id, a name and a width, length & height, this can be used by the gripper
data User = User { uid :: Integer, name :: String, width :: Double, height :: Double, len :: Double} deriving (Generic, Show)
instance ToJSON User
instance FromJSON User

-- this is a message which is relayed back after a characture substitute of . for !
data Msg = Msg { uid :: Integer, message :: String} deriving (Generic, Show)
instance ToJSON Msg
instance FromJSON Msg

data Error = Error { message :: String } deriving (Generic, Show)
instance ToJSON Error
instance FromJSON Error

addUser :: [User] -> User -> [User]
addUser users user = user:users

deleteUser :: [User] -> Integer -> [User]
deleteUser users i = filter (\user -> uid user /= i) users

findUser :: [User] -> Integer -> Maybe User
findUser users i = find (\u -> uid u == i) users

-- check the result was valid
do_case res =
    case res of
        (Just x) -> return x
        Nothing  -> return -99
		
main :: IO ()
main = do
  users <- newIORef [] :: IO (IORef [User])
  -- get the PORT from the port environment e.g. PORT=3000 ; export PORT
  let port = maybe 3000 read <$> lookupEnv "PORT" :: IO Int
  scotty (read port::Int) $ do
    -- $ curl -X GET http://localhost:3000/users
    get "/users" $ do
      let us = liftIO (readIORef users)
      status status200
      json us
    -- $ curl -X GET http://localhost:3000/users/1
    get "/users/:uid" $ do
      let us = liftIO (readIORef users)
      let i = param "uid"
      case findUser us (read i) of
        Just u -> status status200 >> json u
        Nothing -> status status404 >> json (Error ("Not Found uid = " <> i))
    -- $ curl -X GET http://localhost:3000/width/1
    get "/width/:uid" $ do
      let us = liftIO (readIORef users)
      let i = param "uid"
	  let res = findUser us (read i)
      matched_user = do_case res
	  if (matched_user /= -99) then (User { width = c }) = matched_user 
      else status status404
	  if (matched_user /= -99) then status status200 
      else json (Error ("Not Found uid = " <> i))	        		    
      if (matched_user /= -99) then text ("" <> c <> "")        
    -- curl -X POST http://localhost:3000/users -d '{ "uid": 1, "name": "mid_size_box", "width": 34.67, "height": 12.9, "len": 1.12 }'
    post "/users" $ do
      let u = jsonData
      let us = liftIO $ readIORef users
      liftIO $ writeIORef users $ addUser us u
      status status201
      json u
    -- curl -X POST http://localhost:3000/users_modified -d '{ "uid": 1, "name": "mid_size_box", "width": 34.67, "height": 12.9, "len": 1.12 }'
    post "/users_modified" $ do
      let u = jsonData
      let us = liftIO $ readIORef users
	  -- in this example we will add 0.5 to the width
	  let w = width u                                     
	  let g = u { width = (w + 0.5) }
      liftIO $ writeIORef users $ addUser us g
      status status201
      json g
    -- curl -v -X DELETE http://localhost:3000/users/1
    delete "/users/:uid" $ do
      let i = param "uid"
      let us = liftIO $ readIORef users
      liftIO $ writeIORef users $ deleteUser us i
      status status204
    -- $ curl -X GET http://localhost:3000/html
    get "/html" $ do
      status status200
      html "<h1>This is the scotty webserver! <br> writing some html........</h1>"
    -- $ curl -X GET http://localhost:3000/text/charlie
    get "/text/:you" $ do
      let you = param "you"
      let us = liftIO (readIORef users)
      status status200
      text ("Hello " <> you <> ", json is " <> (json us))
    -- curl -X POST http://localhost:3000/msg -d '{ "uid": 1, "message": "this. is the message..." }' changes all . to ! before print
    post "/msg" $ do
      let m = jsonData
      -- b <- message m or below is alternative
	  -- let (Msg a b) = m
	  -- let messge = T.pack b
	  let (Msg a b) = m
	  let messge = T.pack b
	  let mm = T.map (\c -> if c == '.' then '!' else c) messge
	  let ll = T.length messge
      status status200
      text ("length" <> ll <> " original message " <> b <> " changed " <> mm))
    -- curl -X POST http://localhost:3000/fileshow 
    post "/fileshow" $ do
      let bs = L.hGetContents =<< openBinaryFile "/home/mark/haskell/my_file.txt" ReadMode
      status status200
	  text bs
    -- curl -D - http://localhost:8080/redirect/to/root
    get "/redirect/to/root" $ do
      status status302
      setHeader "Haskell" "scotty"
      redirect "/"