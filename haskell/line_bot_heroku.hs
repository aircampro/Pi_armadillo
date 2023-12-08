-- Simple line bot in haskell, it returns a text with a link as you set it in myBotsCorrespondence
-- must have CHANNEL_SECRET="ur_secret_channel"; export CHANNEL_SECRET; CHANNEL_TOKEN="ur_token"; export CHANNEL_TOKEN 

-- cabal file should have 
-- main-is:             Main.hs
--   ghc-options:         -threaded -rtsopts -with-rtsopts=-N
--   build-depends:       base
--                      , line
--                      , text
--                      , wai
--                      , warp
--   default-language:    Haskell2010

-- we are making the bot with crates line,wai,warp
  
-- $ stack new my-linebot
-- $ git init

-- Get a LINE Bot account and create a Heroku app. 

-- If you make a Heroku app by ticking, first set it to the repository remotely.

-- $ heroku git:remote -a <app-name>
-- Then set the buildpack.

-- $ heroku buildpacks:set https://github.com/mfine/heroku-buildpack-stack
-- Then just deploy.

-- $ git push heroku master

{-# LANGUAGE OverloadedStrings #-}

module Main where

import Control.Monad (forM_)
import Data.Maybe(fromJust)
import qualified Data.Text as T (Text, pack, unpack)
import Line.Messaging.API ( APIIO, APIError, ChannelSecret, ChannelAccessToken
                          , Message(..), runAPI, reply)
import Line.Messaging.Webhook ( Event(..), EventMessage(..), ReplyToken(..)
                              , ReplyableEvent(..), webhookApp
                              , defaultOnFailure, getMessage, getReplyToken)
import Line.Messaging.Types (Text(..))
import Network.Wai (Application)
import Network.Wai.Handler.Warp (run)
import System.Environment (lookupEnv)

-- example this is my documentation with examples on the heroku line bot
myBotsCorrespondence :: [([Text], [Text], URL)]
myBotsCorrespondence =
  [ ( ["Says this", "Says this"]]
    , ["Gets this", "First quote", "whats the first one", "Line1", "show line 1", "line one", "show line one"]]
    , "http://link.com/your_pdf_for_this.pdf")
  , ( ["Says that", "Says that"]
    , ["Gets that", "Second quote", "what is the second one", "whats the second one", "line 2 please", "line two", "show line two"]
    , "http://disi.link.com/second_link.pdf")
  , ( ["Bot conversation 3"]
    , ["Gets three", "Third quote", "what is the third one", "whats the 3rd one", "line 3 please", "line three", "show line three"]
    , "http://disi.link.com/third_link.pdf")
  , ( ["speech 4", "speech 4"]
    , ["Get fourth", "Fourth quote", "what is the fourth one", "whats the 4th one", "line 4 please", "line four", "show line four"]
    , "http://www.sciencedirect.com/number_4.pdf")
  , ( ["number 5", "number 5"])
    , ["Gets fifth", "Fifth quote", "whats the 5th one", "Line5", "show line 5", "line five", "show line number five"]]
    , "http://ctp.di.fct.unl.pt/~btoninho/mscs12.pdf")
  ]
  
main :: IO ()
main = do
  port <- maybe 8080 read <$> lookupEnv "PORT" :: IO Int
  run port app

getChannelSecret :: IO ChannelSecret
getChannelSecret = T.pack . fromJust <$> lookupEnv "CHANNEL_SECRET"

getChannelToken :: IO ChannelAccessToken
getChannelToken = T.pack . fromJust <$> lookupEnv "CHANNEL_TOKEN"

-- | A WAI application to handle webhook requests.
app :: Application
app req f = do
  channelSecret <- getChannelSecret
  webhookApp channelSecret handler defaultOnFailure req f

handler :: [Event] -> IO ()
handler events = forM_ events handleEvent

handleEvent :: Event -> IO ()
handleEvent (MessageEvent event) = handleMessageEvent event
handleEvent _ = return ()

handleMessageEvent :: ReplyableEvent EventMessage -> IO ()
handleMessageEvent event = do
  case getMessage event of
    TextEM _ (Text text) -> echo (getReplyToken event) (getCorrespondence text)
    _ -> echo (getReplyToken event) "undefined message"

getCorrespondence :: Text -> Text
getCorrespondence = fromMaybe "unknown..." . lookupCorrespondence

lookupCorrespondence :: Text -> Maybe Text
lookupCorrespondence txt = msum $ fmap match myBotsCorrespondence
  where
    txt' = toLower txt
    match (as, bs, url)
      | any ((==) txt' . toLower) as = appendUrl url <$> safeHead bs
      | any ((==) txt' . toLower) bs = appendUrl url <$> safeHead as
      | otherwise = Nothing

appendUrl :: URL -> Text -> Text
appendUrl url = unwords . (: [url])

safeHead :: [a] -> Maybe a
safeHead = find (const True)

api :: APIIO a -> IO (Either APIError a)
api = runAPI getChannelToken

echo :: ReplyToken -> T.Text -> IO ()
echo replyToken content = do
  api $ reply replyToken [ Message . Text $ content ]
  return ()
  
