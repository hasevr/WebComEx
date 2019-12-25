<html>
    <body>
        <?php
        echo "Request URI = ". $_SERVER['REQUEST_URI']. "<br>\r\n";
        foreach ($_GET as $key => $value){
        echo $key . " = " . $value . "<br>\r\n";
        }
        ?>
    </body>
</html>