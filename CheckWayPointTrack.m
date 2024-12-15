function isSuccessfullyPassed = CheckWayPointTrack(bodyXYZPosition, actualTime, timeForWaypointPasage, wayPoints, positionTolerance)
    % Inicializace
    isSuccessfullyPassed = 1; % Začínáme s předpokladem, že dron úspěšně prošel všemi branami
    numWaypoints = size(wayPoints, 1); % Počet waypointů
    currentWaypointIndex = 1; % Index aktuálního waypointu

    % Iterace přes všechny waypointy
    for i = 1:numWaypoints
        % Získání cílových souřadnic waypointu
        waypoint = wayPoints(i, :);
        
        % Kontrola, zda je čas pro tento waypoint platný
        if actualTime > timeForWaypointPasage(i)
            % Pokud je časový limit pro tento waypoint překročen, dron neprošel branou
            isSuccessfullyPassed = 0;
            return;
        end

        % Simulace pohybu dronu v čase
        while actualTime <= timeForWaypointPasage(i) % Kontrolujeme, zda je čas pro tento waypoint
            % Získání aktuální pozice dronu
            currentX = bodyXYZPosition.X;
            currentY = bodyXYZPosition.Y;
            currentZ = bodyXYZPosition.Z;

            % Vypočítáme vzdálenost od waypointu
            distanceToWaypoint = sqrt((currentX - waypoint(1))^2 + ...
                                      (currentY - waypoint(2))^2 + ...
                                      (currentZ - waypoint(3))^2);

            % Pokud je dron dostatečně blízko waypointu, zaznamenáme čas a přejdeme na další
            if distanceToWaypoint < positionTolerance
                break;
            end

            % Pokud se dron nepohne směrem k waypointu nebo nezmění polohu, vyhodnotíme, že waypoint nebyl proleten
            if actualTime > timeForWaypointPasage(i)
                isSuccessfullyPassed = 0;
                return;
            end

            % Pokud dron neproletí waypoint do stanoveného času, považujeme to za chybu
        end
    end
end