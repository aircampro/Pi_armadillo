-- example webserver using haskell with spock

{-# LANGUAGE MultiWayIf #-}
{-# LANGUAGE OverloadedStrings #-}
module Main where

import Web.Spock
import Web.Spock.Config

import Control.Monad.Trans
import Data.Monoid
import Data.IORef
import qualified Data.Text as T
import System.Environment (lookupEnv)
import Control.Monad.IO.Class (liftIO)

import Network.HTTP.Types.Status

data MySession = EmptySession
data MyAppState = DummyAppState (IORef Int)

-- function to square a number
square = \i -> do
  return $ i * i

-- generate a number from rules 
gen_a_num = \i -> do
  forM [1..30] $ \x -> do
    if
      | x`mod`15 == 0 -> do
        s <- z - y + s
      | x`mod`5  == 0 -> do
        z <- (x+i)+z
      | x`mod`3  == 0 -> do
        y <- y + 1
      | otherwise -> do
        y <- i + x
  return (s + y + z)
		
main :: IO ()
main =
    do ref <- newIORef 0
       spockCfg <- defaultSpockCfg EmptySession PCNoDatabase (DummyAppState ref)
	   -- get the PORT from the port environment e.g. PORT=3000 ; export PORT
       -- port <- getEnv "PORT"
       port <- maybe 8080 read <$> lookupEnv "PORT" :: IO Int
       runSpock (read port::Int) (spock spockCfg app)

app :: SpockM () MySession MyAppState ()
app =
    do get root $
       -- curl http://localhost:8080
           html "<h1>This is the spock webserver!</h1>"
           status status200
           --text "This is the spock webserver!"
       -- curl http://localhost:8080/hello/harry
       get ("hello" <//> var) $ \name ->
           do (DummyAppState ref) <- getState
              visitorNumber <- liftIO $ atomicModifyIORef' ref $ \i -> (i+1, i+1)
              text ("Hello " <> name <> ", you are visitor number " <> T.pack (show visitorNumber))
              status status200
       -- curl http://localhost:8080/square/simon
       get ("square" <//> var) $ \name ->
           do (DummyAppState ref) <- getState
              squareNumber <- liftIO $ atomicModifyIORef' ref $ \i -> (square i, square i)
              text ("Hello " <> name <> ", the number from the square function is now " <> T.pack (show squareNumber))
              status status200
       -- curl http://localhost:8080/generate/graeme			  
       get ("generate" <//> var) $ \name ->
           do (DummyAppState ref) <- getState
              gNumber <- liftIO $ atomicModifyIORef' ref $ \i -> (gen_a_num i, gen_a_num i)
              text ("Hello " <> name <> ", the number from the generate function is now " <> T.pack (show gNumber))
              status status200
			  
			  